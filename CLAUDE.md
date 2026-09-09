# Kontekst projekta

ROS 2 Jazzy workspace za pripremu doktorskog ispita iz SLAM biblioteka
i za istraživanje uticaja izbora back-end solvera pose-graph optimizacije
(Ceres, g2o, GTSAM, SPA) na geometrijsku tačnost oblaka tačaka,
validiranu prema TLS referenci.

Autor: geodeta (TLS, mobilni SLAM, fotogrametrija, izravnanje mreža).
Jaka matematička osnova; ROS terminologiju uvoditi objašnjeno, ne pretpostavljati.

## Okruženje
- Ubuntu 24.04 pod WSL2, ROS 2 Jazzy iz apt-a
- Dve mašine: DESKTOP-K81LV4B (radna, iza UNS proxy-ja), LAPTOP-TRU615A1
- Sinhronizacija preko GitHub-a: istrazivaci00/ros2_ws1

## Pravila
- Kod: Python (rclpy) i C++ (rclcpp). Paketi u `src/`.
- `build/`, `install/`, `log/` nikad ne idu u git.
- Posle izmene Python koda dovoljno je `colcon build --symlink-install`.
- Komentari i poruke commita na srpskom.
- Objašnjavaj *zašto*, ne samo *šta* — cilj je razumevanje, ne gotov kod.