from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI, Request, Form
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
import googlemaps
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import numpy as np
from urllib.parse import urlencode


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Or restrict to your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# class RouteRequest(BaseModel):
#     locations: List[str]
#     distances: List[List[float]]
#
# @app.post("/optimize-route")
# def optimize_route(data: RouteRequest):
#     locations = data.locations
#     dist = data.distances
#     n = len(locations)
#
#     min_cost = float("inf")
#     best_path = []
#
#     for perm in itertools.permutations(range(n)):
#         cost = sum(dist[perm[i]][perm[i+1]] for i in range(n - 1))
#         cost += dist[perm[-1]][perm[0]]
#         if cost < min_cost:
#             min_cost = cost
#             best_path = perm
#
#     return {
#         "route": [locations[i] for i in best_path],
#         "cost": min_cost
#     }

templates = Jinja2Templates(directory="templates")


def compute_optimal_route(api_key, places, distance_matrix=None, mode='walking', avoid='ferries'):
    n = len(places)

    if distance_matrix is None:
        gmaps = googlemaps.Client(key=api_key)
        distance_matrix = np.zeros((n, n))

        for i, origin in enumerate(places):
            for j, destination in enumerate(places):
                if i != j:
                    result = gmaps.distance_matrix(origin, destination, mode=mode, avoid=avoid)
                    try:
                        meters = result['rows'][0]['elements'][0]['distance']['value']
                        distance_matrix[i][j] = meters
                    except Exception as e:
                        raise ValueError(f"Error getting distance between '{origin}' and '{destination}': {e}")
                else:
                    distance_matrix[i][j] = 0

        distance_matrix = distance_matrix.tolist()
    else:
        distance_matrix = np.array(distance_matrix)

    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.SAVINGS
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(60)

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
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

        return {
            "ordered_locations": ordered_places,
            "total_distance_km": round(total_distance / 1000, 2),
            "route_indices": route,
            "distance_matrix": distance_matrix
        }
    else:
        raise Exception("No solution found.")


def split_segments(route_places, max_per_segment=10):
    segments = []
    i = 0
    while i < len(route_places):
        segment = route_places[i:i + max_per_segment]
        if len(segment) >= 2:
            segments.append(segment)
        i += max_per_segment - 1
    return segments


def build_gmaps_url(segment):
    return "https://www.google.com/maps/dir/?" + urlencode({
        'api': 1,
        'origin': segment[0],
        'destination': segment[-1],
        'travelmode': 'walking',
        'waypoints': '|'.join(segment[1:-1]) if len(segment) > 2 else ''
    })


@app.get("/", response_class=HTMLResponse)
async def get_form(request: Request):
    return templates.TemplateResponse("form.html", {"request": request})


@app.post("/optimize", response_class=HTMLResponse)
async def optimize_route(request: Request, api_key: str = Form(...), locations: str = Form(...)):
    location_list = [loc.strip() for loc in locations.split(',') if loc.strip()]
    try:
        result = compute_optimal_route(api_key=api_key, places=location_list)
        segments = split_segments(result['ordered_locations'])
        maps_links = [build_gmaps_url(seg) for seg in segments]
        return templates.TemplateResponse("result.html", {
            "request": request,
            "route": result['ordered_locations'],
            "distance": result['total_distance_km'],
            "maps_links": maps_links
        })
    except Exception as e:
        return templates.TemplateResponse("form.html", {
            "request": request,
            "error": str(e)
        })