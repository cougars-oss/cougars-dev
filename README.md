# 🌊 CoUGARs: Configurable Underwater Group of Autonomous Robots

[![arXiv](https://img.shields.io/badge/arXiv-2511.08822-b31b1b.svg)](https://arxiv.org/pdf/2511.08822)
[![ROS 2 CI](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/ros2_ci.yml)
[![Docker CI](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_ci.yml/badge.svg)](https://github.com/cougars-auv/cougars-dev/actions/workflows/docker_ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/cougars-auv/cougars-dev/main.svg)](https://results.pre-commit.ci/latest/github/cougars-auv/cougars-dev/main)

CoUGARs is a low-cost, configurable AUV platform designed for multi-agent autonomy research by the [Field Robotic Systems Lab (FROST Lab)](https://frostlab.byu.edu) at [Brigham Young University](https://byu.edu).

## 🚀 Get Started

> **Prerequisites:** Several gigabytes of storage, 64-bit Linux or Windows, and a competent NVIDIA GPU (for HoloOcean simulation).

- Install Docker and set up the Linux development environment.

  **Linux (Recommended):**

  - Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).

  **Windows:**

  - Install [WSL2](https://docs.microsoft.com/en-us/windows/wsl/install).

  - Install [Docker Desktop](https://docs.docker.com/desktop/) and enable the [WSL2 backend](https://docs.docker.com/desktop/windows/wsl/).

- Generate and add a [GitHub SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux).

- Clone the `cougars-dev` repository.

  ```bash
  git clone git@github.com:cougars-auv/cougars-dev.git
  ```

- Choose a development workflow:

  **Simulation (HoloOcean):**

  - Build a runtime image for [HoloOcean-ROS](https://github.com/byu-holoocean/holoocean-ros/tree/main/docker). When prompted to run `./build_container.sh`, specify the branch `nelson/fgo-dev` using `./build_container.sh -b nelson/fgo-dev`.

  - Open the `cougars-dev` repository in VSCode. Check for a notification in the bottom right. When it pops up, select "Reopen in Container". If you don't see a notification, open the Command Palette (`Ctrl + Shift + P`), search for "Dev Containers: Reopen in Container", and select it. When prompted to select a devcontainer.json file, choose `CoUGARs Dev (HoloOcean)`.

  - Open a new terminal window using `` Ctrl + Alt + Shift + ` `` and launch a HoloOcean scenario in the `holoocean-ct` container using `./holoocean_launch.sh`.

    ```bash
    cd ~/cougars-dev/scripts && ./holoocean_launch.sh
    ```

  - Open a new terminal, build the `ros2_ws` workspace, and launch the simulation stack using `./sim_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./sim_launch.sh
    ```

  **Recorded Data (`rosbag2`):**

  - Copy your `rosbag2` bag into the `bags` folder at the root of the repository.

  - Open a new terminal window using `` Ctrl + Alt + Shift + ` ``, build the `ros2_ws` workspace, and launch the development stack using `./bag_launch.sh`.

    ```bash
    cd ~/cougars-dev/ros2_ws && colcon build
    cd ~/cougars-dev/scripts && ./bag_launch.sh
    ```

## 🤝 Contributing

- **Create a Branch:** Create a new branch using the format `name/feature` (e.g., `nelson/repo-docs`).

- **Make Changes:** Develop and debug your new feature. Add good documentation.

  > If you need to add dependencies, update the `package.xml`, `Dockerfile`, `cougars.repos`, or `dependencies.repos` in your branch and test building the image locally. The CI will automatically build and push the new image to Docker Hub upon merge.

- **Sync Frequently:** Regularly rebase your branch against `main` (or merge `main` into your branch) to prevent conflicts.

- **Submit a PR:** Open a pull request, ensure required tests pass, and merge once approved.

## 📚 Citations

Please cite our relevant publications if you find this repository useful for your research:

### CoUGARs
```bibtex
@misc{durrant2025lowcostmultiagentfleetacoustic,
  title={Low-cost Multi-agent Fleet for Acoustic Cooperative Localization Research},
  author={Nelson Durrant and Braden Meyers and Matthew McMurray and Clayton Smith and Brighton Anderson and Tristan Hodgins and Kalliyan Velasco and Joshua G. Mangelson},
  year={2025},
  eprint={2511.08822},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.08822},
}
```

### HoloOcean-ROS
```bibtex
@misc{meyers2025testingevaluationunderwatervehicle,
  title={Testing and Evaluation of Underwater Vehicle Using Hardware-In-The-Loop Simulation with HoloOcean},
  author={Braden Meyers and Joshua G. Mangelson},
  year={2025},
  eprint={2511.07687},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.07687},
}
```

### HoloOcean (General Use)
```bibtex
@inproceedings{potokar2022holooceanunderwaterroboticssim,
  author={Easton Potokar and Spencer Ashford and Michael Kaess and Joshua G. Mangelson},
  title={Holo{O}cean: An Underwater Robotics Simulator},
  booktitle={Proc. IEEE Intl. Conf. on Robotics and Automation, ICRA},
  address={Philadelphia, PA, USA},
  month={May},
  year={2022}
}
```

### HoloOcean 2.0 (Features)
```bibtex
@misc{romrell2025previewholoocean20,
  title={A Preview of HoloOcean 2.0},
  author={Blake Romrell and Abigail Austin and Braden Meyers and Ryan Anderson and Carter Noh and Joshua G. Mangelson},
  year={2025},
  eprint={2510.06160},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2510.06160},
}
```
