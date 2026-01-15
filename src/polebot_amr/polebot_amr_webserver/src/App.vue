<template>
  <div id="app">
    <router-view />
  </div>
</template>

<script>
export default {
  name: 'App'
}
</script>

<style>
/* Global Styles */
:root {
  --primary-dark: #0f172a;
  --primary-darker: #020617;
  --sidebar-bg: #1e293b;
  --sidebar-hover: #334155;
  --sidebar-active: #3b82f6;
  --card-bg: #1e293b;
  --card-border: #334155;
  --border-color: #475569;
  --text-primary: #f8fafc;
  --text-secondary: #cbd5e1;
  --text-muted: #94a3b8;
  --accent-blue: #3b82f6;
  --accent-green: #10b981;
  --accent-yellow: #f59e0b;
  --accent-red: #ef4444;
  --accent-purple: #8b5cf6;
  --hover-light: rgba(255, 255, 255, 0.05);
  --shadow-light: rgba(0, 0, 0, 0.1);
  --shadow-medium: rgba(0, 0, 0, 0.3);
  --shadow-heavy: rgba(0, 0, 0, 0.5);
}

* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

body {
  font-family: 'Segoe UI', 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
  background-color: var(--primary-dark);
  color: var(--text-primary);
  overflow: hidden;
  height: 100vh;
}

#app {
  height: 100vh;
  width: 100vw;
}

button {
  font-family: inherit;
  cursor: pointer;
  border: none;
  outline: none;
}

/* Scrollbar Styling */
::-webkit-scrollbar {
  width: 8px;
  height: 8px;
}

::-webkit-scrollbar-track {
  background: #1e293b;
  border-radius: 4px;
}

::-webkit-scrollbar-thumb {
  background: #475569;
  border-radius: 4px;
}

::-webkit-scrollbar-thumb:hover {
  background: #64748b;
}

/* Utility Classes */
.text-gradient {
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.glass-effect {
  background: rgba(30, 41, 59, 0.8);
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.pulse-animation {
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}
</style>