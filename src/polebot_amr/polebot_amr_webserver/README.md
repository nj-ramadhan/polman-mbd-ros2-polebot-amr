# Vue 3 + TypeScript + Vite

This template should help get you started developing with Vue 3 and TypeScript in Vite. The template uses Vue 3 `<script setup>` SFCs, check out the [script setup docs](https://v3.vuejs.org/api/sfc-script-setup.html#sfc-script-setup) to learn more.

Learn more about the recommended Project Setup and IDE Support in the [Vue Docs TypeScript Guide](https://vuejs.org/guide/typescript/overview.html#project-setup).

1. Choose a version (e.g., Node.js 20.x, 22.x, etc.). As of Dec 2025, Node.js 22.x (LTS) is current LTS.

For Node.js 22.x (LTS):

bash
    curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -

🔔 Replace 22.x with 20.x or 18.x if you need an older LTS.

2. Install Node.js and npm
bash
    sudo apt install -y nodejs
3. Verify
bash
    node --version   # e.g., v22.12.0
    npm --version    # e.g., 10.9.0

🔄 Update Node.js (if already installed via NodeSource)
Just run:

bash
    sudo apt update && sudo apt upgrade nodejs


Run Ros Bridge
    sudo apt install ros-jazzy-rosbridge-server

    ros2 run rosbridge_server rosbridge_websocket
