from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import itertools

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Or restrict to your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class RouteRequest(BaseModel):
    locations: List[str]
    distances: List[List[float]]

@app.post("/optimize-route")
def optimize_route(data: RouteRequest):
    locations = data.locations
    dist = data.distances
    n = len(locations)

    min_cost = float("inf")
    best_path = []

    for perm in itertools.permutations(range(n)):
        cost = sum(dist[perm[i]][perm[i+1]] for i in range(n - 1))
        cost += dist[perm[-1]][perm[0]]
        if cost < min_cost:
            min_cost = cost
            best_path = perm

    return {
        "route": [locations[i] for i in best_path],
        "cost": min_cost
    }