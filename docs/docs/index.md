# BlueOC Robot Fulfillment

**Two robots and a web server that deliver a cup of water — end to end, with no
human in the loop.**

A customer scans a QR code at their table and orders a drink. A central
orchestrator queues the task and coordinates two robots: a **robotic arm** fills
a cup and loads it onto a **mobile robot**, which navigates the room and delivers
it to the table. When the cup is empty, the same loop runs in reverse to refill it.

`ROS 2` · `Isaac Sim` · `MoveIt 2` · `Nav2` · `FastAPI + HTMX`

---

## The vision

Most robot demos do *one* thing well. The hard, interesting problem is
**coordination** — making a manipulator and a mobile base cooperate on a task
that neither can finish alone, driven by a real customer request rather than a
scripted trigger.

This project is that system, built end to end:

- **A real user surface.** A QR-code deep link and a live task screen — the order
  comes from a person, not a terminal.
- **A central brain.** One orchestrator owns the task queue, plans the legs of
  each delivery, and dispatches both robots over ROS 2.
- **Two very different robots.** A 5-DOF arm that must *see*, *grasp*, and *place*;
  a car-like base that must *map*, *localize*, and *navigate*. They meet at the
  station to hand a cup between them.
- **Simulation first.** Everything runs against NVIDIA Isaac Sim, so the whole
  pipeline is developed and validated before a single motor turns.

---

## The three pieces

<div class="grid cards" markdown>

-   **Robot Arm (RA) — SO-ARM 101**

    A 5-DOF arm with a single-jaw gripper. It detects cups with an overhead
    camera, visually servos onto each one, carries it to a dispenser to fill, and
    places it on the tray.

    [**Overview →**](ra_concepts.md) · [Setup](ra_setup.md) · [Pick & Place](ra_pick_and_place.md)

-   **JetRacer (AMR) — mobile base**

    A car-like (Ackermann) robot. It maps the space with SLAM, localizes with
    Nav2, and shuttles between the station and the tables, docking precisely at
    each one.

    [**Overview →**](amr_concepts.md) · [Setup](amr_setup.md) · [Navigate & Deliver](amr_pick_and_place.md)

-   **Orchestrator — the brain**

    A FastAPI + HTMX server. It serves the customer's QR page, owns the order
    queue, and dispatches both robots over ROS 2 — the only component that knows
    about *the task* rather than *the robot*.

    [**Overview →**](orchestrator.md) · [Combined solution](solution_pick_and_deliver.md)

</div>

---

## How a delivery works

1. **Order** — the customer scans the QR at their table and requests water.
2. **Queue** — the orchestrator enqueues the task and reserves a robot.
3. **Fill** — the arm detects a cup, grasps it, fills it at the dispenser.
4. **Hand off** — the arm places the cup on the AMR's tray at the station.
5. **Deliver** — the AMR navigates to the table and docks.
6. **Refill** — an empty cup is collected and the loop runs again.

See the full message flows in [Pick and Deliver](solution_pick_and_deliver.md).

---

## Where the project stands

!!! info "Phase 1 — Proof of Concept"
    Both robots run **in simulation** (Isaac Sim). The arm completes the full
    detect → grasp → fill → place loop; the AMR maps, navigates, and docks. There
    is **no on-device firmware yet** — the workstation drives the simulator, and
    the same ROS 2 topic contract will drive the real hardware.

    The orchestrator ↔ robot integration is the current frontier: the AMR docking
    seam and the arm's task topics are specified and partially wired.

Each robot's page is honest about what is **built**, what is **unverified**, and
what is **planned** — start with the overviews above.

---

## Start here

| I want to… | Go to |
|---|---|
| Understand the concepts before touching code | [Get Started](GET-STARTED.md) |
| Run the robot arm | [RA Setup Guide](ra_setup.md) |
| Run the JetRacer | [AMR Setup Guide](amr_setup.md) |
| See how both robots + the server fit together | [Pick and Deliver](solution_pick_and_deliver.md) |
| Look up a topic, route, or parameter | [API Book](api/index.md) |