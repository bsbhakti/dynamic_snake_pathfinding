scenarios = [
    {'agents': [(0, 0), (4, 4)],'goals': [(3, 3), (0, 4)],'dynamic_obstacles': [{'position': (2, 2), 'start_time': 3, 'duration': 2}],'grid': [[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],[0, 1, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0]]},
    {"agents": [(0, 0), (8, 8)], "goals": [(8, 8), (0, 0)],"dynamic_obstacles": [{"position": (3, 3), "start_time": 4, "duration": 3},{"position": (7, 7), "start_time": 5, "duration": 3}, {"position": (2, 2), "start_time": 3, "duration": 2},],"grid": [[0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 1, 1, 1, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 1, 0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 1, 0, 1, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 1, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0, 0, 0],],},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (3, 4), 'start_time': 5, 'duration': 3}], 'grid': [[0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (0, 4)], 'goals': [(0, 4), (1, 4)], 'dynamic_obstacles': [{'position': (0, 1), 'start_time': 1, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0], [1, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 1, 0, 0], [1, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (2, 4), 'start_time': 0, 'duration': 2}], 'grid': [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 1], [1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (3, 4), 'start_time': 2, 'duration': 2}], 'grid': [[0, 0, 0, 0, 1, 1], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 1, 1, 0, 0, 0], [0, 0, 0, 0, 1, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (3, 2), 'start_time': 4, 'duration': 1}], 'grid': [[0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (0, 3), 'start_time': 0, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 1, 1, 0], [0, 0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (1, 4), 'start_time': 4, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (3, 5), 'start_time': 1, 'duration': 2}], 'grid': [[0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (1, 4), 'start_time': 5, 'duration': 2}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (0, 1), 'start_time': 4, 'duration': 2}], 'grid': [[0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (1, 2), 'start_time': 1, 'duration': 3}], 'grid': [[0, 1, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (2, 2), 'start_time': 1, 'duration': 1}], 'grid': [[0, 0, 0, 1, 1, 0], [0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (3, 5), 'start_time': 0, 'duration': 2}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (3, 4), 'start_time': 4, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0], [1, 0, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (1, 4), 'start_time': 2, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (1, 4), 'start_time': 0, 'duration': 1}], 'grid': [[0, 0, 0, 1, 0, 0], [0, 0, 1, 0, 1, 0], [1, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (6, 1), 'start_time': 0, 'duration': 2}], 'grid': [[0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (4, 5), 'start_time': 1, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (1, 0), 'start_time': 1, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0], [0, 0, 1, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (5, 0), 'start_time': 5, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (5, 1), 'start_time': 3, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0], [1, 0, 0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (2, 1), 'start_time': 0, 'duration': 2}], 'grid': [[0, 0, 1, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (5, 0), 'start_time': 3, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (2, 1), 'start_time': 0, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0], [0, 1, 0, 0, 1], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (3, 4), 'start_time': 2, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 1, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (4, 0), 'start_time': 4, 'duration': 2}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (2, 6), 'start_time': 3, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0, 1, 0], [0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 1, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (1, 1), 'start_time': 0, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (0, 1), 'start_time': 3, 'duration': 1}], 'grid': [[0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (2, 1), 'start_time': 2, 'duration': 3}], 'grid': [[0, 0, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 1, 0, 0], [0, 0, 0, 0, 0], [0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (0, 2), 'start_time': 5, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0, 0, 1], [0, 1, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0, 0], [1, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (3, 1), 'start_time': 4, 'duration': 2}], 'grid': [[0, 0, 1, 0, 0], [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [1, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (5, 4), 'start_time': 5, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 1, 0, 0], [0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (2, 4), 'start_time': 0, 'duration': 1}], 'grid': [[0, 0, 0, 1, 0], [0, 0, 1, 0, 0], [1, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (4, 2), 'start_time': 3, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (4, 0), 'start_time': 3, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 1, 1, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (6, 3), 'start_time': 1, 'duration': 2}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 1, 0], [1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (0, 0), 'start_time': 5, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 1, 1, 0, 0, 0], [0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (2, 3), 'start_time': 1, 'duration': 1}], 'grid': [[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (0, 0), 'start_time': 2, 'duration': 1}], 'grid': [[0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 1, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (2, 4), 'start_time': 4, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 1, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (3, 4), 'start_time': 0, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 1, 0, 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (1, 2), 'start_time': 2, 'duration': 2}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [1, 0, 0, 0, 0], [0, 0, 0, 0, 0], [1, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (1, 0), 'start_time': 1, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 0], [1, 0, 1, 0, 0, 0], [0, 1, 0, 0, 0, 0], [1, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(5, 5), (0, 0)], 'dynamic_obstacles': [{'position': (0, 0), 'start_time': 4, 'duration': 1}], 'grid': [[0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0], [1, 0, 1, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (6, 6)], 'goals': [(6, 6), (0, 0)], 'dynamic_obstacles': [{'position': (4, 5), 'start_time': 4, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 1, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (4, 0), 'start_time': 1, 'duration': 3}], 'grid': [[0, 0, 0, 0, 0], [1, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 1], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (4, 4)], 'goals': [(4, 4), (0, 0)], 'dynamic_obstacles': [{'position': (0, 0), 'start_time': 3, 'duration': 2}], 'grid': [[0, 0, 0, 0, 0], [0, 0, 1, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]},
    {'agents': [(0, 0), (5, 5)], 'goals': [(4, 4), (1, 1)], 'dynamic_obstacles': [{'position': (3, 3), 'start_time': 4, 'duration': 2}],'grid': [[0, 0, 0, 0, 0, 1],[0, 0, 0, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 1, 0, 0, 0],[1, 0, 0, 0, 0, 0]]},
    {'agents': [(0, 1), (7, 6)], 'goals': [(6, 6), (1, 7)], 'dynamic_obstacles': [{'position': (4, 4), 'start_time': 6, 'duration': 3},{'position': (2, 5), 'start_time': 2, 'duration': 2}],
        'grid': [
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0, 0],
            [1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ]
    },
    
    {
        'agents': [(0, 0), (9, 9)],  # Agent 1 starts at (0,0), Agent 2 starts at (9,9)
        'goals': [(8, 2), (2, 8)],   # Goals: Agent 1 -> (8,2), Agent 2 -> (2,8)
        'dynamic_obstacles': [
            {'position': (4, 5), 'start_time': 5, 'duration': 4},
            {'position': (7, 3), 'start_time': 8, 'duration': 3},
            {'position': (3, 8), 'start_time': 3, 'duration': 2}
        ],
        'grid': [
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
            [1, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 1, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 1, 0, 0]
        ]
    },
    # {
    #     'agents': [(0, 0), (4, 4)],
    #     'goals': [(3, 3), (0, 4)],
    #     'dynamic_obstacles': [
    #         {'position': (2, 2), 'start_time': 3, 'duration': 2}
    #     ],
    #     'grid': [
    #         [0, 0, 0, 0, 0],
    #         [0, 0, 1, 0, 0],
    #         [0, 1, 0, 0, 0],
    #         [0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0]
    #     ]
    # },

    # {
    #     'agents': [(0, 0), (11, 11)],  # Agent 1 starts at (0,0), Agent 2 starts at (11,11)
    #     'goals': [(10, 3), (3, 10)],   # Goals: Agent 1 -> (10,3), Agent 2 -> (3,10)
    #     'dynamic_obstacles': [
    #         {'position': (5, 6), 'start_time': 4, 'duration': 3},
    #         {'position': (8, 8), 'start_time': 7, 'duration': 4},
    #         {'position': (6, 2), 'start_time': 10, 'duration': 3}
    #     ],
    #     'grid': [
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    #         [0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
    #         [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0],
    #         [0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    #         [0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0],
    #         [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    #         [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
    #         [0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0]
    #     ]
    # },
    ]
