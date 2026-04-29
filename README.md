# IoT Data Processing System for Distance Estimation (LoRa ToF)

This project presents the design, implementation, and evaluation of a data acquisition and processing system for distance estimation using LoRa communication and Time of Flight (ToF) measurements.

Originally developed as an undergraduate thesis, the project explores the feasibility of ToF-based ranging using low-cost hardware, while focusing on handling noisy, high-variability data in real-world conditions.

---

## Overview

The system is based on a bidirectional communication protocol between two ESP32 devices equipped with LoRa (RFM95) modules. It captures timestamped signals to estimate round-trip time (RTT) and compute distances.

Due to hardware and system limitations, raw measurements present significant noise (jitter) and inconsistencies, making data processing and statistical filtering a core part of the solution.

---

## Key Features

* Development of a custom telemetry protocol for synchronized data exchange.
* High-frequency timestamp capture for RTT measurement.
* Processing of noisy experimental data with high variability.
* Data filtering using statistical methods (grouping, standard deviation minimization).
* Comparative analysis between ToF and RSSI-based distance estimation.
* Evaluation across multiple environments (indoor and outdoor scenarios).

---

## Data Engineering Perspective

This project can be understood as a data pipeline:

1. **Data Acquisition**
   Sensor measurements captured from distributed IoT devices.

2. **Data Processing**
   Cleaning, filtering, and grouping of noisy data.

3. **Data Validation**
   Selection of reliable measurements using statistical criteria.

4. **Data Analysis**
   Comparison of estimation methods and evaluation of accuracy.

The main challenge addressed is ensuring data quality under hardware-induced noise and timing inconsistencies.

---

## Tech Stack

* C / C++ (embedded firmware development).
* ESP32.
* LoRa (RFM95 modules).
* SPI communication.
* Python (optional: for data analysis, if applicable).
* Statistical data processing.

---

## Results & Findings

* High variability (jitter) in RTT measurements significantly affects accuracy.
* System-level latencies introduce large errors in ToF-based distance estimation.
* ToF results were not reliable for precise ranging under tested conditions.
* RSSI-based estimation, despite its limitations, showed more consistent behavior.
* Environmental factors and hardware constraints strongly impact data quality.

---

## Key Learnings

* Real-world data is often noisy, incomplete, and inconsistent.
* Data cleaning and filtering are critical for extracting meaningful insights.
* Hardware constraints can dominate system performance.
* Statistical methods are essential when working with unreliable measurements.

---

## Future Improvements

* Integration with a structured data pipeline (e.g., SQL / BigQuery).
* Automated data ingestion and storage.
* Visualization and exploratory data analysis.
* Real-time processing and monitoring.

---

## Author

Gustavo Araya
Computer Engineer

---

## Notes

This project is experimental and aims to evaluate feasibility rather than provide a production-ready solution. The results highlight the challenges of working with real-world data and low-cost hardware in distributed systems.
