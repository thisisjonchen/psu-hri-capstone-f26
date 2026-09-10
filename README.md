# Project

A multi-drone system for autonomously searching indoor environments, such as stadiums, to identify and localize lost items.

The drones do not retrieve objects. Instead, they search assigned areas, detect candidate lost items, estimate their locations, and report them to a shared system for later retrieval.

## System Overview

```text
Venue Map
   ↓
Multi-Drone Area Assignment
   ↓
Coverage Path Planning
   ↓
Autonomous Drone Search
   ↓
Object Detection & Tracking
   ↓
Lost Item Localization
   ↓
Shared Lost & Found Database
   ↓
Operator Dashboard / Retrieval
```

## Perception


The perception subsystem is responsible for:

* detecting candidate lost items
* classifying and tracking objects
* reducing duplicate detections
* determining whether objects may be abandoned
* estimating object locations within the environment

## Planning

The planning subsystem is responsible for:

* dividing the search area between drones
* generating efficient coverage paths
* minimizing duplicate coverage
* avoiding obstacles and other drones
* dynamically replanning when conditions change

## Software Stack

* **Python** — primary autonomy and perception language
* **React / TypeScript** — operator dashboard
