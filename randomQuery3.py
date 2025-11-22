import json
import random
import os

def create_phase3_query_file(graph_filename,
                             num_orders,
                             num_delivery_agents,
                             output_filename):
    """
    Create a Phase-3 queries.json with structure:

    {
      "meta": { "id": "phase3_test" },
      "events": [
        {
          "orders": [ ... ],
          "fleet": {
            "num_delievery_guys": ...,
            "depot_node": 0
          }
        }
      ]
    }
    """

    # ---- Ensure directory exists (if any) ----
    out_dir = os.path.dirname(output_filename)
    if out_dir:  # avoid os.makedirs("") when no directory part
        os.makedirs(out_dir, exist_ok=True)

    # ---- Load graph.json ----
    with open(graph_filename, "r") as f:
        graph = json.load(f)

    # Extract node IDs either from "nodes" list or meta.nodes
    if "nodes" in graph and isinstance(graph["nodes"], list):
        node_ids = [n["id"] for n in graph["nodes"]]
    else:
        n = graph["meta"]["nodes"]
        node_ids = list(range(n))

    depot_node = 0
    if depot_node not in node_ids:
        raise ValueError(f"Depot node {depot_node} not in graph nodes")

    usable_nodes = [x for x in node_ids if x != depot_node]
    if len(usable_nodes) < 2:
        raise ValueError("Need at least 2 non-depot nodes for pickup/dropoff")

    # ---- Generate orders ----
    orders = []
    for oid in range(1, num_orders + 1):
        p = random.choice(usable_nodes)
        d = random.choice(usable_nodes)
        while d == p:
            d = random.choice(usable_nodes)

        orders.append({
            "order_id": oid,
            "pickup": p,
            "dropoff": d
        })

    # ---- Wrap inside events[] with meta ----
    data = {
        "meta": {
            "id": "phase3_test"
        },
        "events": [
            {
                "orders": orders,
                "fleet": {
                    # keep the typo exactly as in spec
                    "num_delievery_guys": num_delivery_agents,
                    "depot_node": depot_node
                }
            }
        ]
    }

    # ---- Write queries.json ----
    with open(output_filename, "w") as f:
        json.dump(data, f, indent=2)

    print(f"Created Phase-3 queries file at {output_filename} "
          f"with {num_orders} orders and {num_delivery_agents} drivers.")


create_phase3_query_file(
    graph_filename=f"testcases/graphs/real_graph_large2.json",
    num_orders=10,
    num_delivery_agents=1,
    output_filename=f"testcases/queries_3/queries_1_real.json"
)

for i in range(2, 6):
    create_phase3_query_file(
        graph_filename=f"testcases/graphs/real_graph_large2.json",
        num_orders=100 * i,
        num_delivery_agents=10 * i,
        output_filename=f"testcases/queries_3/queries_{i}_real.json"
    )