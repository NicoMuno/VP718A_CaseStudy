from utils.demo_simulation import Simulation

sim = Simulation()
sim.init()

try:
    while sim.running:
        sim.tick()
except KeyboardInterrupt:
    print("[Main] Ctrl+C received")

sim.end()
