<template>
  <div class="app-layout">
    <!-- Sidebar -->
    <div class="sidebar" :class="{ collapsed: sidebarCollapsed }">
      <div class="sidebar-header">
        <div class="logo">
          <i class="fas fa-robot"></i>
          <h2 v-show="!sidebarCollapsed">NAV2 Controller</h2>
        </div>
      </div>
      <nav class="sidebar-nav">
        <router-link to="/map" class="nav-item" :title="sidebarCollapsed ? 'Goal Planning' : ''">
          <span class="nav-icon"><i class="fas fa-map-marker-alt"></i></span>
          <span class="nav-text" v-show="!sidebarCollapsed">Goal Planning</span>
        </router-link>
        <router-link to="/mission" class="nav-item" :title="sidebarCollapsed ? 'Mission Control' : ''">
          <span class="nav-icon"><i class="fas fa-rocket"></i></span>
          <span class="nav-text" v-show="!sidebarCollapsed">Mission Control</span>
        </router-link>
        <div class="nav-divider" v-show="!sidebarCollapsed"></div>
        <router-link to="/settings" class="nav-item" :title="sidebarCollapsed ? 'Settings' : ''">
          <span class="nav-icon"><i class="fas fa-cog"></i></span>
          <span class="nav-text" v-show="!sidebarCollapsed">Settings</span>
        </router-link>
        <router-link to="/logs" class="nav-item" :title="sidebarCollapsed ? 'System Logs' : ''">
          <span class="nav-icon"><i class="fas fa-clipboard-list"></i></span>
          <span class="nav-text" v-show="!sidebarCollapsed">System Logs</span>
        </router-link>
      </nav>
      <div class="sidebar-footer">
        <div class="connection-status" :class="connectionStatus.class">
          <div class="status-indicator"></div>
          <div class="status-info" v-show="!sidebarCollapsed">
            <div class="status-message">{{ connectionStatus.message }}</div>
            <div class="status-detail">ROS Bridge</div>
          </div>
          <div class="status-icon" v-show="sidebarCollapsed">
            <i :class="connectionStatus.icon"></i>
          </div>
        </div>
      </div>
    </div>

    <!-- Main Content Area -->
    <div class="main-content" :style="{ marginLeft: sidebarMarginLeft }">
      <!-- Navbar -->
      <div class="navbar">
        <div class="navbar-left">
          <div class="breadcrumb">
            <h1 class="page-title">{{ currentPageTitle }}</h1>
            <div class="page-subtitle">Navigation Control System</div>
          </div>
        </div>
        
        <div class="navbar-right">
          <!-- Hamburger Button (Desktop) -->
          <button class="menu-toggle desktop-only" @click="toggleSidebar">
            <span class="menu-icon">
              <i :class="sidebarCollapsed ? 'fas fa-chevron-right' : 'fas fa-chevron-left'"></i>
            </span>
          </button>
          
          <!-- Hamburger Button (Mobile) -->
          <button class="menu-toggle mobile-only" @click="toggleSidebar">
            <span class="menu-icon">
              <i :class="sidebarCollapsed ? 'fas fa-bars' : 'fas fa-times'"></i>
            </span>
          </button>
          
          <div class="user-info">
            <div class="user-details">
              <span class="user-name">Operator</span>
              <span class="user-role">Administrator</span>
            </div>
            <div class="user-avatar">
              <i class="fas fa-user-shield"></i>
            </div>
          </div>
        </div>
      </div>

      <!-- Content Area -->
      <div class="content-area">
        <router-view 
          :is-connected="isConnected"
          :ros="ros"
          :goals="goals"
          :map-data="mapData"
          :map-info="mapInfo"
          :connection-status="connectionStatus"
          @update-goals="updateGoals"
        />
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'

export default {
  name: 'AppLayout',
  data() {
    return {
      currentPageTitle: 'Goal Planning',
      sidebarCollapsed: false,
      ros: null,
      isConnected: false,
      rosBridgeUrl: `ws://${window.location.hostname}:9090`,
      // Map data
      mapInfo: null,
      mapData: null,
      mapTopic: null,

      // Goals data
      goals: [],

      connectionStatus: {
        message: 'Disconnected',
        class: 'disconnected',
        icon: 'fas fa-plug'
      },
      
      // Flag untuk kontrol responsive
      isMobile: false
    }
  },

  computed: {
    sidebarWidth() {
      return this.sidebarCollapsed ? 70 : 260
    },
    
    sidebarMarginLeft() {
      // Di mobile, sidebar sebagai overlay (tidak ada margin)
      if (this.isMobile) {
        return '0'
      }
      return `${this.sidebarWidth}px`
    }
  },

  watch: {
    '$route'(to) {
      this.updatePageTitle(to)
    }
  },

  mounted() {
    this.connectROS()
    this.updatePageTitle(this.$route)
    
    // Listen for resize events
    window.addEventListener('resize', this.handleResize)
    // Initial check
    this.checkIfMobile()
  },

  beforeUnmount() {
    window.removeEventListener('resize', this.handleResize)
  },

  methods: {
    toggleSidebar() {
      this.sidebarCollapsed = !this.sidebarCollapsed
      
      // Trigger resize event untuk komponen yang perlu update
      setTimeout(() => {
        window.dispatchEvent(new Event('resize'))
      }, 300)
    },

    checkIfMobile() {
      this.isMobile = window.innerWidth <= 1024
      
      // Hanya set sidebar collapsed default di mobile saat pertama kali
      if (this.isMobile && !localStorage.getItem('sidebarInitialized')) {
        this.sidebarCollapsed = true
        localStorage.setItem('sidebarInitialized', 'true')
      }
    },

    handleResize() {
      this.checkIfMobile()
    },

    updatePageTitle(route) {
      const routeTitles = {
        '/map': 'Goal Planning',
        '/mission': 'Mission Control',
        '/settings': 'Settings',
        '/logs': 'System Logs'
      }
      this.currentPageTitle = routeTitles[route.path] || 'NAV2 Controller'
    },

    // ROS Connection
    connectROS() {
      this.ros = new ROSLIB.Ros({ url: this.rosBridgeUrl })

      this.ros.on('connection', () => {
        this.isConnected = true
        this.updateStatus('Connected to ROS Bridge', 'connected', 'fas fa-link')
        this.subscribeToMap()
      })

      this.ros.on('error', (error) => {
        this.isConnected = false
        this.updateStatus('Connection error', 'error', 'fas fa-unlink')
        console.error('ROS Error:', error)
      })

      this.ros.on('close', () => {
        this.isConnected = false
        this.updateStatus('Disconnected', 'disconnected', 'fas fa-plug')
      })
    },

    updateStatus(message, statusClass, icon = '') {
      this.connectionStatus = {
        message,
        class: statusClass,
        icon: icon || this.getDefaultIcon(statusClass)
      }
    },

    getDefaultIcon(statusClass) {
      const icons = {
        'connected': 'fas fa-check-circle',
        'disconnected': 'fas fa-exclamation-circle',
        'error': 'fas fa-times-circle'
      }
      return icons[statusClass] || 'fas fa-circle'
    },

    subscribeToMap() {
      if (!this.ros) return

      this.mapTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/map',
        messageType: 'nav_msgs/msg/OccupancyGrid'
      })

      this.mapTopic.subscribe(this.handleMapMessage)
      this.updateStatus('Map subscribed', 'connected', 'fas fa-map')
    },

    handleMapMessage(mapData) {
      console.log('🗺️ Map data received')
      this.mapData = mapData

      const origin = mapData.info.origin.position
      const resolution = mapData.info.resolution
      const width = mapData.info.width
      const height = mapData.info.height

      this.mapInfo = {
        width: width,
        height: height,
        resolution: resolution,
        origin: { x: origin.x, y: origin.y }
      }
    },

    // Goals Management
    updateGoals(newGoals) {
      this.goals = newGoals
      console.log('Goals updated:', this.goals.length)
    }
  }
}
</script>

<style scoped>
/* Main Layout */
.app-layout {
  display: flex;
  min-height: 100vh;
  background-color: var(--primary-dark);
  color: var(--text-primary);
  overflow-x: hidden;
}

/* Sidebar */
.sidebar {
  width: 260px;
  background: linear-gradient(180deg, var(--sidebar-bg) 0%, var(--primary-darker) 100%);
  border-right: 1px solid var(--border-color);
  display: flex;
  flex-direction: column;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  position: fixed;
  left: 0;
  top: 0;
  bottom: 0;
  z-index: 100;
  box-shadow: 4px 0 12px var(--shadow-heavy);
  overflow-y: auto;
  overflow-x: hidden;
}

.sidebar.collapsed {
  width: 70px;
}

.sidebar-header {
  padding: 24px 20px;
  border-bottom: 1px solid var(--border-color);
  flex-shrink: 0;
}

.logo {
  display: flex;
  align-items: center;
  gap: 12px;
}

.logo i {
  font-size: 24px;
  color: var(--accent-blue);
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  flex-shrink: 0;
}

.logo h2 {
  margin: 0;
  font-size: 18px;
  font-weight: 700;
  background: linear-gradient(135deg, var(--text-primary), var(--text-secondary));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.sidebar-nav {
  flex: 1;
  padding: 20px 0;
  display: flex;
  flex-direction: column;
  gap: 4px;
  overflow-y: auto;
  overflow-x: hidden;
}

.nav-item {
  display: flex;
  align-items: center;
  padding: 14px 20px;
  margin: 0 8px;
  color: var(--text-secondary);
  text-decoration: none;
  transition: all 0.2s ease;
  border-radius: 8px;
  position: relative;
  overflow: hidden;
  white-space: nowrap;
}

.nav-item:hover {
  background-color: var(--sidebar-hover);
  color: var(--text-primary);
  transform: translateX(4px);
}

.nav-item:hover .nav-icon {
  transform: scale(1.1);
}

.nav-item.router-link-active {
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  color: white;
  box-shadow: 0 4px 12px rgba(59, 130, 246, 0.3);
}

.nav-item.router-link-active .nav-icon {
  color: white;
}

.nav-item.router-link-active::before {
  content: '';
  position: absolute;
  left: 0;
  top: 0;
  height: 100%;
  width: 4px;
  background: var(--accent-yellow);
  border-radius: 0 4px 4px 0;
}

.nav-icon {
  font-size: 18px;
  margin-right: 12px;
  width: 24px;
  text-align: center;
  transition: transform 0.2s ease;
  flex-shrink: 0;
}

.nav-text {
  font-weight: 500;
  font-size: 14px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  transition: opacity 0.3s ease;
}

.nav-divider {
  height: 1px;
  background: linear-gradient(90deg, transparent, var(--border-color), transparent);
  margin: 16px 20px;
  flex-shrink: 0;
}

.sidebar-footer {
  padding: 20px;
  border-top: 1px solid var(--border-color);
  flex-shrink: 0;
}

.connection-status {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px;
  border-radius: 8px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  transition: all 0.3s ease;
  overflow: hidden;
}

.connection-status.connected {
  background-color: rgba(16, 185, 129, 0.1);
  border-color: rgba(16, 185, 129, 0.3);
}

.connection-status.disconnected {
  background-color: rgba(245, 158, 11, 0.1);
  border-color: rgba(245, 158, 11, 0.3);
}

.connection-status.error {
  background-color: rgba(239, 68, 68, 0.1);
  border-color: rgba(239, 68, 68, 0.3);
}

.status-indicator {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  background-color: var(--text-muted);
  flex-shrink: 0;
}

.connection-status.connected .status-indicator {
  background-color: var(--accent-green);
  box-shadow: 0 0 10px var(--accent-green);
  animation: pulse 2s infinite;
}

.connection-status.disconnected .status-indicator {
  background-color: var(--accent-yellow);
}

.connection-status.error .status-indicator {
  background-color: var(--accent-red);
}

.status-info {
  flex: 1;
  min-width: 0;
  overflow: hidden;
}

.status-message {
  font-weight: 600;
  font-size: 13px;
  color: var(--text-primary);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.status-detail {
  font-size: 11px;
  color: var(--text-muted);
  margin-top: 2px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.status-icon {
  font-size: 16px;
  flex-shrink: 0;
}

/* Main Content Area */
.main-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: auto;
  height: 100vh;
  background: linear-gradient(135deg, var(--primary-dark) 0%, #1a1f2e 100%);
  margin-left: 260px;
  transition: margin-left 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  width: calc(100% - 260px);
}

/* Ketika sidebar collapsed */
.sidebar.collapsed + .main-content {
  margin-left: 70px;
  width: calc(100% - 70px);
}

/* Navbar */
.navbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0 24px;
  height: 70px;
  background-color: rgba(30, 41, 59, 0.8);
  backdrop-filter: blur(10px);
  border-bottom: 1px solid var(--border-color);
  flex-shrink: 0;
  z-index: 50;
  box-shadow: 0 4px 6px -1px var(--shadow-heavy);
  position: sticky;
  top: 0;
}

.navbar-left {
  display: flex;
  align-items: center;
  gap: 20px;
  flex: 1;
  min-width: 0;
}

.breadcrumb {
  display: flex;
  flex-direction: column;
  gap: 4px;
  min-width: 0;
}

.page-title {
  margin: 0;
  font-size: 22px;
  font-weight: 700;
  background: linear-gradient(135deg, var(--text-primary), var(--accent-blue));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.page-subtitle {
  font-size: 12px;
  color: var(--text-muted);
  font-weight: 500;
  letter-spacing: 0.5px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.navbar-right {
  display: flex;
  align-items: center;
  gap: 12px;
  flex-shrink: 0;
}

/* Hamburger Button Styles */
.menu-toggle {
  width: 40px;
  height: 40px;
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  border: none;
  color: white;
  font-size: 16px;
  cursor: pointer;
  border-radius: 10px;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.3s ease;
  position: relative;
  z-index: 101;
  flex-shrink: 0;
}

/* Desktop-only hamburger */
.menu-toggle.desktop-only {
  display: flex;
  order: 1;
}

/* Mobile-only hamburger */
.menu-toggle.mobile-only {
  display: none;
  order: -1;
}

.menu-toggle:hover {
  transform: scale(1.1);
  box-shadow: 0 6px 20px rgba(59, 130, 246, 0.4);
}

.menu-toggle:active {
  transform: scale(0.95);
}

.menu-icon i {
  transition: transform 0.3s ease;
}

.menu-toggle:hover .menu-icon i {
  transform: scale(1.2);
}

.user-info {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 8px 16px;
  background-color: rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  border: 1px solid var(--border-color);
  transition: all 0.3s ease;
  flex-shrink: 0;
}

.user-info:hover {
  background-color: rgba(255, 255, 255, 0.1);
  transform: translateY(-2px);
}

.user-details {
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  min-width: 0;
}

.user-name {
  font-weight: 600;
  color: var(--text-primary);
  font-size: 14px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.user-role {
  font-size: 11px;
  color: var(--accent-blue);
  font-weight: 500;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.user-avatar {
  width: 36px;
  height: 36px;
  border-radius: 50%;
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 16px;
  color: white;
  box-shadow: 0 4px 12px rgba(59, 130, 246, 0.3);
  flex-shrink: 0;
}

/* Content Area */
.content-area {
  flex: 1;
  padding: 24px;
  overflow-y: auto;
  position: relative;
  transition: padding 0.3s ease;
}

/* Responsive design */
@media (max-width: 1024px) {
  .sidebar {
    transform: translateX(-100%);
    width: 260px !important;
    box-shadow: 4px 0 20px rgba(0, 0, 0, 0.3);
    z-index: 1000;
  }
  
  .sidebar:not(.collapsed) {
    transform: translateX(0);
  }
  
  .sidebar.collapsed {
    transform: translateX(-100%);
  }
  
  .main-content {
    margin-left: 0 !important;
    width: 100% !important;
  }
  
  /* Sembunyikan hamburger desktop di mobile */
  .menu-toggle.desktop-only {
    display: none;
  }
  
  /* Tampilkan hamburger mobile */
  .menu-toggle.mobile-only {
    display: flex;
    order: -1;
    margin-right: 12px;
    background: var(--primary-dark);
    border: 1px solid var(--border-color);
    color: var(--text-primary);
  }
  
  .menu-toggle.mobile-only:hover {
    background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
    color: white;
  }
  
  /* Overlay untuk menutup sidebar saat diklik di luar */
  .sidebar:not(.collapsed)::before {
    content: '';
    position: fixed;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    background-color: rgba(0, 0, 0, 0.5);
    z-index: -1;
  }
}

@media (max-width: 768px) {
  .navbar {
    padding: 0 16px;
    height: 60px;
  }
  
  .page-title {
    font-size: 18px;
  }
  
  .user-details {
    display: none;
  }
  
  .content-area {
    padding: 16px;
  }
  
  .sidebar {
    width: 85%;
    max-width: 300px;
  }
  
  .menu-toggle.mobile-only {
    width: 36px;
    height: 36px;
    font-size: 14px;
  }
  
  .navbar-right {
    gap: 8px;
  }
}

@media (max-width: 480px) {
  .navbar {
    padding: 0 12px;
  }
  
  .menu-toggle.mobile-only {
    width: 34px;
    height: 34px;
    font-size: 13px;
  }
  
  .content-area {
    padding: 12px;
  }
  
  .page-title {
    font-size: 16px;
  }
  
  .user-avatar {
    width: 32px;
    height: 32px;
    font-size: 14px;
  }
}

/* Animations */
@keyframes pulse {
  0%, 100% {
    opacity: 1;
  }
  50% {
    opacity: 0.5;
  }
}

/* Smooth transitions */
.sidebar,
.main-content,
.content-area {
  will-change: width, margin-left, transform;
}

/* Scrollbar styling */
.sidebar::-webkit-scrollbar,
.content-area::-webkit-scrollbar {
  width: 6px;
}

.sidebar::-webkit-scrollbar-track,
.content-area::-webkit-scrollbar-track {
  background: transparent;
}

.sidebar::-webkit-scrollbar-thumb,
.content-area::-webkit-scrollbar-thumb {
  background: var(--border-color);
  border-radius: 3px;
}

.sidebar::-webkit-scrollbar-thumb:hover,
.content-area::-webkit-scrollbar-thumb:hover {
  background: var(--text-muted);
}

/* Touch device optimization */
@media (hover: none) and (pointer: coarse) {
  .nav-item:hover {
    transform: none;
  }
  
  .menu-toggle:hover {
    transform: none;
  }
  
  .user-info:hover {
    transform: none;
  }
  
  .nav-item:active,
  .menu-toggle:active,
  .user-info:active {
    transform: scale(0.98);
  }
}
</style>