from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
import googlemaps
import numpy as np
from urllib.parse import urlencode
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

app = FastAPI()

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # adjust for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Models ---
class RouteRequest(BaseModel):
    api_key: str
    locations: List[str]

class RouteResponse(BaseModel):
    ordered_locations: List[str]
    total_distance_km: float
    maps_links: List[str]

# --- Google Maps Helper ---
def get_distance_matrix(api_key: str, places: List[str], mode='walking', avoid='ferries'):
    gmaps = googlemaps.Client(key=api_key)
    n = len(places)
    matrix = np.zeros((n, n))

    for i, origin in enumerate(places):
        for j, destination in enumerate(places):
            if i != j:
                try:
                    result = gmaps.distance_matrix(origin, destination, mode=mode, avoid=avoid)
                    matrix[i][j] = result['rows'][0]['elements'][0]['distance']['value']
                except Exception as e:
                    raise HTTPException(status_code=400, detail=f"Failed to get distance between {origin} and {destination}: {e}")
    return matrix

# --- Route Optimizer ---
def solve_tsp(distance_matrix, places):
    n = len(places)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_idx, to_idx):
        return int(distance_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)])

    transit_cb_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.SAVINGS
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.FromSeconds(60)

    solution = routing.SolveWithParameters(search_params)

    if not solution:
        raise HTTPException(status_code=500, detail="No optimal solution found")

    index = routing.Start(0)
    route = []
    total_distance = 0

    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        route.append(node)
        next_index = solution.Value(routing.NextVar(index))
        total_distance += routing.GetArcCostForVehicle(index, next_index, 0)
        index = next_index

    route.append(manager.IndexToNode(index))
    ordered_places = [places[i] for i in route]
    return ordered_places, total_distance / 1000

# --- Google Maps Segment Links ---
def split_segments(locations, max_len=10):
    i, segments = 0, []
    while i < len(locations):
        segment = locations[i:i+max_len]
        segments.append(segment)
        i += max_len - 1
    return segments

def build_gmaps_url(segment):
    return "https://www.google.com/maps/dir/?" + urlencode({
        'api': 1,
        'origin': segment[0],
        'destination': segment[-1],
        'travelmode': 'walking',
        'waypoints': '|'.join(segment[1:-1]) if len(segment) > 2 else ''
    })

# --- API Endpoints ---
@app.get("/")
def health_check():
    return {"message": "Backend is running"}

@app.post("/optimize-route", response_model=RouteResponse)
def optimize_route(request: RouteRequest):
    places = request.locations
    if len(places) < 2:
        raise HTTPException(status_code=400, detail="At least two locations required")

    matrix = get_distance_matrix(api_key=request.api_key, places=places)
    route, total_km = solve_tsp(matrix, places)
    segments = split_segments(route)
    map_links = [build_gmaps_url(seg) for seg in segments]

    return {
        "ordered_locations": route,
        "total_distance_km": round(total_km, 2),
        "maps_links": map_links
    }