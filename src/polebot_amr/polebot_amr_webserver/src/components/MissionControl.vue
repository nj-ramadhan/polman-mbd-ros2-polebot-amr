<template>
  <div class="mission-control">
    <!-- Header Section -->
    <div class="header-section">
      <div class="header-content">
        <div class="header-icon">
          <i class="fas fa-rocket"></i>
        </div>
        <div class="header-text">
          <h1>Mission Control</h1>
          <p class="subtitle">Create, manage, and execute navigation missions with real-time updates</p>
        </div>
      </div>
      <div class="header-actions">
        <button class="header-btn" @click="refreshData" :disabled="isLoading">
          <i class="fas fa-sync-alt" :class="{ 'fa-spin': isLoading }"></i>
          <span class="btn-text">{{ isLoading ? 'Loading...' : 'Refresh' }}</span>
        </button>
        <button class="header-btn primary" @click="showNewMissionModal">
          <i class="fas fa-plus"></i>
          <span class="btn-text">New Mission</span>
        </button>
      </div>
    </div>

    <!-- Main Content Grid -->
    <div class="main-grid">
      <!-- Left Column: Mission Categories & Goals -->
      <div class="column left-column">
        <!-- Mission Categories Panel -->
        <div class="card categories-panel">
          <div class="panel-header">
            <div class="header-left">
              <h3><i class="fas fa-folder"></i> Categories</h3>
              <span class="count-badge">{{ categories.length }}</span>
            </div>
            <button class="icon-btn" @click="showNewCategoryModal" title="Add Category">
              <i class="fas fa-plus"></i>
            </button>
          </div>

          <div class="categories-list">
            <div v-for="category in categories" :key="category.id" class="category-item"
              :class="{ active: selectedCategory === category.id }" @click="selectCategory(category.id)">
              <div class="category-info">
                <div class="category-name">
                  <i :class="category.icon || 'fas fa-folder'"></i>
                  {{ category.name }}
                </div>
                <div class="category-stats">
                  <span class="mission-count">
                    <i class="fas fa-rocket"></i>
                    {{ category.mission_count || 0 }}
                  </span>
                </div>
              </div>
              <div class="category-actions">
                <button class="action-btn" @click.stop="editCategory(category)" title="Edit">
                  <i class="fas fa-edit"></i>
                </button>
                <button class="action-btn danger" @click.stop="deleteCategory(category.id)" title="Delete">
                  <i class="fas fa-trash"></i>
                </button>
              </div>
            </div>

            <div v-if="categories.length === 0" class="empty-state">
              <i class="fas fa-folder-open"></i>
              <p>No categories yet</p>
              <button class="action-btn small" @click="showNewCategoryModal">
                <i class="fas fa-plus"></i> Add Category
              </button>
            </div>
          </div>
        </div>

        <!-- Available Goals Panel -->
        <div class="card goals-panel">
          <div class="panel-header">
            <div class="header-left">
              <h3><i class="fas fa-bullseye"></i> Available Goals</h3>
              <span class="count-badge">{{ filteredGoals.length }}</span>
            </div>
            <div class="filter-controls">
              <select v-model="selectedGoalSet" class="filter-select" @change="onGoalSetChange">
                <option value="">All Goal Sets</option>
                <option v-for="goalSet in goalSets" :key="goalSet.id" :value="goalSet.id">
                  {{ goalSet.set_name || `Set ${goalSet.id}` }}
                </option>
              </select>
              <div class="search-box">
                <i class="fas fa-search"></i>
                <input type="text" v-model="goalSearch" placeholder="Search..." class="search-input">
              </div>
            </div>
          </div>

          <div class="goals-list">
            <!-- Goal Set Info -->
            <div v-if="selectedGoalSet" class="goal-set-info">
              <div class="info-badge">
                <i class="fas fa-layer-group"></i>
                <span>Goal Set: {{ selectedGoalSetName }}</span>
                <button class="clear-btn" @click="clearGoalSetSelection">
                  <i class="fas fa-times"></i>
                </button>
              </div>
            </div>

            <div v-for="goal in paginatedGoals" :key="goal.id" class="goal-item">
              <div class="goal-info">
                <div class="goal-header">
                  <span class="goal-name">{{ goal.set_name || `Goal Set ${goal.goal_set_id}` }}</span>
                  <span class="goal-badge">#{{ goal.sequence_number || 1 }}</span>
                </div>
                <div class="goal-coordinates">
                  <span class="coord-item">
                    <i class="fas fa-long-arrow-alt-right"></i>
                    X: {{ formatCoordinate(goal.position_x) }}
                  </span>
                  <span class="coord-item">
                    <i class="fas fa-long-arrow-alt-up"></i>
                    Y: {{ formatCoordinate(goal.position_y) }}
                  </span>
                </div>
                <div class="goal-meta">
                  <span class="goal-set">
                    <i class="fas fa-layer-group"></i>
                    Map: {{ goal.map_name || 'Unknown' }}
                  </span>
                  <span class="goal-tolerance">
                    <i class="fas fa-bullseye"></i>
                    Tol: {{ formatCoordinate(goal.tolerance_xy) }}
                  </span>
                </div>
              </div>
              <div class="goal-actions">
                <button class="action-btn" @click="previewGoal(goal)" title="Preview">
                  <i class="fas fa-eye"></i>
                </button>
                <button class="action-btn success" @click="addGoalToCurrentMission(goal)"
                  :disabled="!selectedMission || isGoalInMission(goal.id)" title="Add to Mission">
                  <i class="fas fa-plus"></i>
                </button>
              </div>
            </div>

            <div v-if="filteredGoals.length === 0" class="empty-state">
              <i class="fas fa-crosshairs"></i>
              <p v-if="isLoadingGoals">Loading goals...</p>
              <p v-else>No goals available</p>
              <router-link to="/map" class="action-btn small primary">
                <i class="fas fa-map"></i> Go to Planning
              </router-link>
            </div>

            <!-- Pagination -->
            <div v-if="filteredGoals.length > itemsPerPage" class="pagination">
              <button @click="prevPage" :disabled="currentPage === 1" class="pagination-btn">
                <i class="fas fa-chevron-left"></i>
              </button>
              <span class="page-info">Page {{ currentPage }} of {{ totalPages }}</span>
              <button @click="nextPage" :disabled="currentPage === totalPages" class="pagination-btn">
                <i class="fas fa-chevron-right"></i>
              </button>
            </div>
          </div>
        </div>
      </div>

      <!-- Middle Column: Mission Builder -->
      <div class="column middle-column">
        <!-- Current Mission Panel -->
        <div class="card mission-builder">
          <div class="panel-header">
            <div class="header-left">
              <h3><i class="fas fa-tasks"></i> Mission Builder</h3>
              <span class="count-badge">{{ currentMission.goals.length }}</span>
            </div>
            <div class="mission-controls">
              <button class="action-btn" @click="saveMission" :disabled="!canSaveMission">
                <i class="fas fa-save"></i> <span class="btn-text">Save</span>
              </button>
              <button class="action-btn primary" @click="startMission" :disabled="!canStartMission">
                <i class="fas fa-play"></i> <span class="btn-text">Start</span>
              </button>
            </div>
          </div>

          <!-- Mission Info -->
          <div class="mission-info-card">
            <div class="info-row">
              <div class="info-group">
                <label>Mission Name</label>
                <input type="text" v-model="currentMission.name" placeholder="Enter mission name" class="mission-input">
              </div>
              <div class="info-group">
                <label>Category</label>
                <select v-model="currentMission.category_id" class="mission-select">
                  <option value="">Select Category</option>
                  <option v-for="cat in categories" :key="cat.id" :value="cat.id">
                    {{ cat.name }}
                  </option>
                </select>
              </div>
            </div>
            <div class="info-group">
              <label>Description</label>
              <textarea v-model="currentMission.description" placeholder="Describe your mission..."
                class="mission-textarea" rows="2"></textarea>
            </div>
          </div>

          <!-- Mission Goals Sequence -->
          <div class="mission-sequence">
            <div class="sequence-header">
              <h4>Mission Sequence</h4>
              <div class="sequence-actions">
                <button class="action-btn small" @click="clearMissionSequence"
                  :disabled="currentMission.goals.length === 0">
                  <i class="fas fa-trash"></i> <span class="btn-text">Clear</span>
                </button>
                <button class="action-btn small" @click="optimizeSequence" :disabled="currentMission.goals.length < 2">
                  <i class="fas fa-magic"></i> <span class="btn-text">Optimize</span>
                </button>
              </div>
            </div>

            <draggable v-model="currentMission.goals" handle=".drag-handle" item-key="id" class="sequence-list">
              <template #item="{ element: goal, index }">
                <div class="sequence-item">
                  <div class="seq-item-header">
                    <div class="seq-number">
                      <span class="step-badge">{{ index + 1 }}</span>
                      <span class="drag-handle"><i class="fas fa-grip-vertical"></i></span>
                    </div>
                    <div class="seq-info">
                      <div class="seq-name">{{ goal.set_name || `Goal ${goal.sequence_number || 1}` }}</div>
                      <div class="seq-coords">
                        X: {{ formatCoordinate(goal.position_x) }}, Y: {{ formatCoordinate(goal.position_y) }}
                      </div>
                    </div>
                    <div class="seq-actions">
                      <button @click="moveGoalUp(index)" :disabled="index === 0" class="icon-btn">
                        <i class="fas fa-arrow-up"></i>
                      </button>
                      <button @click="moveGoalDown(index)" :disabled="index === currentMission.goals.length - 1"
                        class="icon-btn">
                        <i class="fas fa-arrow-down"></i>
                      </button>
                      <button @click="removeGoalFromMission(index)" class="icon-btn danger">
                        <i class="fas fa-times"></i>
                      </button>
                    </div>
                  </div>

                  <!-- TRIGGER INPUT untuk setiap goal -->
                  <div class="goal-trigger-section">
                    <div class="trigger-header">
                      <span class="trigger-label">Next Goal Trigger</span>
                      <span class="trigger-hint">(Condition to proceed to next goal)</span>
                    </div>
                    <div class="trigger-options">
                      <div class="trigger-option">
                        <input type="radio" :id="'trigger-auto-' + goal.id" value="auto"
                          v-model="goal.next_goal_trigger" class="trigger-radio">
                        <label :for="'trigger-auto-' + goal.id" class="trigger-label-option">
                          <i class="fas fa-robot"></i>
                          <span>Auto (Default)</span>
                          <span class="trigger-desc">Proceed when goal is reached</span>
                        </label>
                      </div>

                      <div class="trigger-option">
                        <input type="radio" :id="'trigger-timer-' + goal.id" value="timer"
                          v-model="goal.next_goal_trigger" class="trigger-radio">
                        <label :for="'trigger-timer-' + goal.id" class="trigger-label-option">
                          <i class="fas fa-clock"></i>
                          <span>Timer</span>
                          <span class="trigger-desc">Wait for specified time</span>
                        </label>

                        <div v-if="goal.next_goal_trigger === 'timer'" class="timer-input">
                          <input type="number" v-model.number="goal.wait_time" min="1" max="300" class="time-input"
                            placeholder="Seconds">
                          <span class="time-unit">seconds</span>
                        </div>
                      </div>

                      <div class="trigger-option">
                        <input type="radio" :id="'trigger-manual-' + goal.id" value="manual"
                          v-model="goal.next_goal_trigger" class="trigger-radio">
                        <label :for="'trigger-manual-' + goal.id" class="trigger-label-option">
                          <i class="fas fa-hand-paper"></i>
                          <span>Manual</span>
                          <span class="trigger-desc">Wait for user confirmation</span>
                        </label>
                      </div>

                      <div class="trigger-option">
                        <input type="radio" :id="'trigger-sensor-' + goal.id" value="sensor"
                          v-model="goal.next_goal_trigger" class="trigger-radio">
                        <label :for="'trigger-sensor-' + goal.id" class="trigger-label-option">
                          <i class="fas fa-thermometer-half"></i>
                          <span>Sensor Condition</span>
                          <span class="trigger-desc">Based on sensor reading</span>
                        </label>

                        <div v-if="goal.next_goal_trigger === 'sensor'" class="sensor-input">
                          <select v-model="goal.sensor_type" class="sensor-select">
                            <option value="">Select Sensor</option>
                            <option value="distance">Distance Sensor</option>
                            <option value="temperature">Temperature</option>
                            <option value="proximity">Proximity</option>
                            <option value="custom">Custom Condition</option>
                          </select>

                          <div v-if="goal.sensor_type" class="sensor-params">
                            <input type="text" v-model="goal.sensor_condition"
                              placeholder="Condition (e.g., distance < 0.5)" class="condition-input">
                          </div>
                        </div>
                      </div>
                    </div>

                    <!-- Advanced trigger settings -->
                    <div v-if="goal.next_goal_trigger && goal.next_goal_trigger !== 'auto'" class="advanced-trigger">
                      <div class="advanced-toggle" @click="goal.showAdvanced = !goal.showAdvanced">
                        <span>Advanced Settings</span>
                        <i :class="['fas', goal.showAdvanced ? 'fa-chevron-up' : 'fa-chevron-down']"></i>
                      </div>

                      <div v-if="goal.showAdvanced" class="advanced-content">
                        <div class="form-group">
                          <label>Timeout (seconds)</label>
                          <input type="number" v-model.number="goal.timeout" min="10" max="600" class="timeout-input"
                            placeholder="Optional">
                        </div>

                        <div class="form-group">
                          <label>Retry Count</label>
                          <input type="number" v-model.number="goal.retry_count" min="0" max="5" class="retry-input"
                            placeholder="0-5">
                        </div>

                        <div class="form-group">
                          <label>On Failure Action</label>
                          <select v-model="goal.on_failure" class="failure-select">
                            <option value="skip">Skip to next</option>
                            <option value="retry">Retry</option>
                            <option value="stop">Stop mission</option>
                            <option value="pause">Pause and wait</option>
                          </select>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </template>
            </draggable>

            <div v-if="currentMission.goals.length === 0" class="empty-sequence">
              <i class="fas fa-list-ol"></i>
              <p>No goals in sequence. Add goals from the left panel.</p>
            </div>
          </div>

          <!-- Mission Statistics -->
          <div class="mission-stats">
            <div class="stat-card">
              <div class="stat-icon success">
                <i class="fas fa-bullseye"></i>
              </div>
              <div class="stat-content">
                <div class="stat-value">{{ currentMission.goals.length }}</div>
                <div class="stat-label">Total Goals</div>
              </div>
            </div>
            <div class="stat-card">
              <div class="stat-icon warning">
                <i class="fas fa-route"></i>
              </div>
              <div class="stat-content">
                <div class="stat-value">{{ calculateTotalDistance().toFixed(2) }}m</div>
                <div class="stat-label">Total Distance</div>
              </div>
            </div>
            <div class="stat-card">
              <div class="stat-icon info">
                <i class="fas fa-clock"></i>
              </div>
              <div class="stat-content">
                <div class="stat-value">{{ estimateMissionTime() }}</div>
                <div class="stat-label">Estimated Time</div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- Right Column: Mission List & Execution -->
      <div class="column right-column">
        <!-- Saved Missions Panel -->
        <div class="card missions-panel">
          <div class="panel-header">
            <div class="header-left">
              <h3><i class="fas fa-rocket"></i> Missions</h3>
              <span class="count-badge">{{ filteredMissions.length }}</span>
            </div>
            <div class="filter-controls">
              <select v-model="missionFilter" class="filter-select">
                <option value="all">All</option>
                <option value="recent">Recent</option>
                <option value="with-goals">With Goals</option>
              </select>
            </div>
          </div>

          <div class="missions-list">
            <div v-for="mission in filteredMissions" :key="mission.id" class="mission-item"
              :class="{ active: selectedMission?.id === mission.id }" @click="loadMission(mission)">
              <div class="mission-info">
                <div class="mission-header">
                  <span class="mission-name">{{ mission.name }}</span>
                  <span class="mission-status">{{ mission.goal_count > 0 ? 'Ready' : 'Empty' }}</span>
                </div>
                <div class="mission-details">
                  <span class="mission-category">
                    <i :class="getCategoryIcon(mission.category_id)"></i>
                    {{ getCategoryName(mission.category_id) }}
                  </span>
                  <span class="mission-goals">
                    <i class="fas fa-bullseye"></i>
                    {{ mission.goal_count || 0 }}
                  </span>
                </div>
                <div class="mission-date">
                  <i class="far fa-calendar"></i>
                  {{ formatDate(mission.created_at) }}
                </div>
              </div>
              <div class="mission-actions">
                <button class="action-btn success" @click.stop="executeMission(mission)"
                  :disabled="!isConnected || missionState.isActive" title="Execute">
                  <i class="fas fa-play"></i>
                </button>
                <button class="action-btn danger" @click.stop="deleteMission(mission.id)" title="Delete">
                  <i class="fas fa-trash"></i>
                </button>
              </div>
            </div>

            <div v-if="filteredMissions.length === 0" class="empty-state">
              <i class="fas fa-rocket"></i>
              <p>No missions created yet</p>
              <button class="action-btn small" @click="showNewMissionModal">
                <i class="fas fa-plus"></i> Create Mission
              </button>
            </div>
          </div>
        </div>

        <!-- Mission Execution Panel -->
        <div class="card execution-panel">
          <div class="panel-header">
            <div class="header-left">
              <h3><i class="fas fa-play-circle"></i> Execution</h3>
              <span :class="['status-badge', missionState.status]">{{ missionState.status }}</span>
            </div>
          </div>

          <div class="execution-content">
            <div class="progress-container">
              <div class="progress-header">
                <span>Progress</span>
                <span>{{ missionProgress }}%</span>
              </div>
              <div class="progress-bar">
                <div class="progress-fill" :style="{ width: missionProgress + '%' }"></div>
              </div>
            </div>

            <div class="execution-info">
              <div class="execution-stats">
                <div class="stat-item">
                  <div class="stat-label">Step</div>
                  <div class="stat-value">{{ missionState.currentStep || 0 }}/{{ missionState.totalSteps }}</div>
                </div>
                <div class="stat-item">
                  <div class="stat-label">Completed</div>
                  <div class="stat-value">{{ missionState.completedGoals }}/{{ missionState.totalSteps }}</div>
                </div>
              </div>

              <!-- Current Goal Info -->
              <div v-if="currentGoal && missionState.isActive" class="current-goal-info">
                <div class="goal-title">
                  <span>Current Goal</span>
                  <span class="step-badge">#{{ missionState.currentStep }}</span>
                </div>
                <div class="goal-coords">
                  <span class="coord">X: {{ formatCoordinate(currentGoal.position_x) }}</span>
                  <span class="coord">Y: {{ formatCoordinate(currentGoal.position_y) }}</span>
                </div>
                <div v-if="currentGoal.next_goal_trigger && currentGoal.next_goal_trigger !== 'auto'"
                  class="trigger-info">
                  <i class="fas fa-clock"></i>
                  <span>Waiting for: {{ currentGoal.next_goal_trigger }}</span>
                  <button v-if="currentGoal.next_goal_trigger === 'manual' && currentTriggerState.manualConfirmation"
                    class="small-btn" @click="confirmManualTrigger">
                    Confirm
                  </button>
                </div>
              </div>
            </div>

            <!-- Execution Controls dengan Tombol Run All Missions -->
            <div class="execution-controls">
              <!-- Tombol Run All Missions -->
              <div class="control-row">
                <button @click="runAllMissions" :disabled="!canRunAllMissions" class="control-btn primary"
                  title="Run All Missions">
                  <i class="fas fa-play-circle"></i>
                  <span class="btn-text">Run All Missions</span>
                </button>
              </div>

              <div class="control-row">
                <button @click="pauseMission" :disabled="!missionState.isActive || missionState.isPaused"
                  class="control-btn warning" title="Pause">
                  <i class="fas fa-pause"></i>
                  <span class="btn-text">Pause</span>
                </button>
                <button @click="resumeMission" :disabled="!missionState.isActive || !missionState.isPaused"
                  class="control-btn success" title="Resume">
                  <i class="fas fa-play"></i>
                  <span class="btn-text">Resume</span>
                </button>
                <button @click="stopMission" :disabled="!missionState.isActive && !isRunningAllMissions"
                  class="control-btn danger" title="Stop">
                  <i class="fas fa-stop"></i>
                  <span class="btn-text">Stop</span>
                </button>
              </div>
              <div class="control-row" v-if="missionState.isActive">
                <button @click="skipToNextGoal" :disabled="!missionState.isActive" class="control-btn small"
                  title="Skip">
                  <i class="fas fa-forward"></i>
                </button>
                <button @click="retryCurrentGoal" :disabled="!missionState.isActive" class="control-btn small"
                  title="Retry">
                  <i class="fas fa-redo"></i>
                </button>
                <button @click="cancelMission" :disabled="!missionState.isActive" class="control-btn small danger"
                  title="Cancel">
                  <i class="fas fa-times"></i>
                </button>
              </div>
            </div>
          </div>
        </div>

        <!-- Mission Logs Panel -->
        <div class="card logs-panel">
          <div class="panel-header">
            <div class="header-left">
              <h3><i class="fas fa-clipboard-list"></i> Logs</h3>
              <span class="count-badge">{{ missionLogs.length }}</span>
            </div>
            <button class="action-btn small" @click="clearLogs" title="Clear Logs">
              <i class="fas fa-trash"></i>
            </button>
          </div>

          <div class="logs-container">
            <div v-for="(log, index) in missionLogs.slice(0, 10)" :key="index" :class="['log-entry', log.type]">
              <span class="log-icon">
                <i :class="getLogIcon(log.type)"></i>
              </span>
              <span class="log-time">{{ log.time }}</span>
              <span class="log-message">{{ log.message }}</span>
            </div>

            <div v-if="missionLogs.length === 0" class="empty-logs">
              <i class="fas fa-info-circle"></i>
              <p>No logs yet</p>
            </div>

            <!-- Show more indicator -->
            <div v-if="missionLogs.length > 10" class="more-logs">
              <span>{{ missionLogs.length - 10 }} more logs...</span>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Manual Confirmation Modal -->
    <div v-if="currentTriggerState.manualConfirmation" class="manual-confirmation-overlay"
      @click.self="currentTriggerState.manualConfirmation = false">
      <div class="manual-confirmation-modal">
        <h3 class="confirmation-title">
          <i class="fas fa-hand-paper"></i> Manual Confirmation Required
        </h3>
        <p class="confirmation-message">
          Step {{ missionState.currentStep }} is waiting for your confirmation to proceed to the next goal.
        </p>

        <div class="current-goal-info"
          style="margin: 15px 0; padding: 10px; background: rgba(255,255,255,0.05); border-radius: 8px;">
          <div style="font-size: 14px; font-weight: 600; margin-bottom: 5px;">Current Goal:</div>
          <div style="font-size: 12px;">
            {{ currentGoal?.set_name || `Goal ${currentGoal?.sequence_number}` }}
          </div>
          <div style="font-size: 11px; color: var(--text-muted); margin-top: 5px;">
            X: {{ formatCoordinate(currentGoal?.position_x) }}, Y: {{ formatCoordinate(currentGoal?.position_y) }}
          </div>
        </div>

        <div class="confirmation-buttons">
          <button class="confirm-btn success" @click="confirmManualTrigger">
            <i class="fas fa-check"></i> Confirm & Continue
          </button>
          <button class="confirm-btn danger" @click="skipToNextGoal">
            <i class="fas fa-forward"></i> Skip This Goal
          </button>
        </div>

        <div style="margin-top: 15px; font-size: 11px; color: var(--text-muted);">
          <i class="fas fa-info-circle"></i> This modal will close automatically when confirmed
        </div>
      </div>
    </div>

    <!-- Modals -->
    <!-- New Mission Modal -->
    <div v-if="showMissionModal" class="modal-overlay" @click.self="closeModal">
      <div class="modal-content">
        <div class="modal-header">
          <h3><i class="fas fa-plus"></i> Create New Mission</h3>
          <button class="modal-close" @click="closeModal">&times;</button>
        </div>
        <div class="modal-body">
          <div class="form-group">
            <label>Mission Name</label>
            <input type="text" v-model="newMission.name" placeholder="Enter mission name" class="modal-input"
              @keyup.enter="createNewMission">
          </div>
          <div class="form-group">
            <label>Category</label>
            <select v-model="newMission.category_id" class="modal-select">
              <option value="">Select Category</option>
              <option v-for="cat in categories" :key="cat.id" :value="cat.id">
                {{ cat.name }}
              </option>
            </select>
          </div>
          <div class="form-group">
            <label>Description (Optional)</label>
            <textarea v-model="newMission.description" placeholder="Enter mission description..." class="modal-input"
              rows="3"></textarea>
          </div>
        </div>
        <div class="modal-footer">
          <button class="modal-btn secondary" @click="closeModal">Cancel</button>
          <button class="modal-btn primary" @click="createNewMission" :disabled="!canCreateMission">
            <i class="fas fa-plus"></i> Create Mission
          </button>
        </div>
      </div>
    </div>

    <!-- New Category Modal -->
    <div v-if="showCategoryModal" class="modal-overlay" @click.self="closeModal">
      <div class="modal-content">
        <div class="modal-header">
          <h3><i class="fas fa-folder-plus"></i> {{ editingCategory ? 'Edit' : 'New' }} Category</h3>
          <button class="modal-close" @click="closeModal">&times;</button>
        </div>
        <div class="modal-body">
          <div class="form-group">
            <label>Category Name *</label>
            <input type="text" v-model="newCategory.name" placeholder="Enter category name" class="modal-input"
              @keyup.enter="saveCategory">
          </div>
          <div class="form-group">
            <label>Description (Optional)</label>
            <textarea v-model="newCategory.description" placeholder="Enter category description..." class="modal-input"
              rows="2"></textarea>
          </div>
          <div class="form-group">
            <label>Icon</label>
            <div class="icon-selection">
              <div v-for="icon in availableIcons" :key="icon"
                :class="['icon-option', { selected: newCategory.icon === icon }]" @click="newCategory.icon = icon">
                <i :class="icon"></i>
              </div>
            </div>
          </div>
          <div class="form-group">
            <label>Color</label>
            <input type="color" v-model="newCategory.color" class="color-picker">
          </div>
        </div>
        <div class="modal-footer">
          <button class="modal-btn secondary" @click="closeModal">Cancel</button>
          <button class="modal-btn primary" @click="saveCategory" :disabled="!canSaveCategory">
            <i class="fas fa-save"></i> {{ editingCategory ? 'Update' : 'Create' }}
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import draggable from 'vuedraggable'
import ROSLIB from 'roslib'

export default {
  name: 'MissionControl',
  components: {
    draggable
  },
  props: {
    isConnected: Boolean,
    ros: Object,
    connectionStatus: {
      type: Object,
      required: true,
      default: () => ({
        message: 'Disconnected',
        class: 'disconnected',
        icon: 'fas fa-plug'
      })
    }
  },
  data() {
    return {
      isLoading: false,
      isLoadingGoals: false,
      categories: [],
      selectedCategory: null,

      // Goal set filtering
      goalSets: [],
      selectedGoalSet: null,
      selectedGoalSetName: '',
      allGoals: [],
      goalSearch: '',
      currentPage: 1,
      itemsPerPage: 10,

      missions: [],
      missionFilter: 'all',
      selectedMission: null,
      currentMission: {
        id: null,
        name: '',
        description: '',
        category_id: '',
        goals: []
      },

      missionState: {
        status: 'idle',
        isActive: false,
        isPaused: false,
        currentStep: 0,
        totalSteps: 0,
        completedGoals: 0,
        startedAt: null,
        elapsedTime: '00:00'
      },

      currentGoal: null,
      currentGoalTime: 0,
      goalStartTime: null,

      // Trigger state
      currentTriggerState: {
        type: 'auto',
        waiting: false,
        timer: null,
        manualConfirmation: false,
        sensorValue: null,
        timeoutTimer: null
      },
      manualConfirmCallback: null,

      missionLogs: [],
      showMissionModal: false,
      showCategoryModal: false,
      editingCategory: false,
      newMission: {
        name: '',
        description: '',
        category_id: ''
      },
      newCategory: {
        id: null,
        name: '',
        description: '',
        icon: 'fas fa-folder',
        color: '#3b82f6'
      },
      availableIcons: [
        'fas fa-folder',
        'fas fa-rocket',
        'fas fa-map-marker-alt',
        'fas fa-route',
        'fas fa-flag-checkered',
        'fas fa-shipping-fast',
        'fas fa-robot',
        'fas fa-home',
        'fas fa-warehouse',
        'fas fa-industry'
      ],

      // ROS topics
      goalTopic: null,
      goalStatusTopic: null,
      completionTimer: null,
      timeInterval: null,

      // Batch missions
      isRunningAllMissions: false,
      batchMissions: [],
      currentBatchMissionIndex: 0,
      batchCompletionCheck: null
    }
  },
  computed: {
    filteredGoals() {
      let filtered = this.allGoals

      // Filter by selected goal set
      if (this.selectedGoalSet) {
        filtered = filtered.filter(goal => goal.goal_set_id === this.selectedGoalSet)
      }

      // Filter by search term
      if (this.goalSearch) {
        const search = this.goalSearch.toLowerCase()
        filtered = filtered.filter(goal =>
          (goal.set_name && goal.set_name.toLowerCase().includes(search)) ||
          (goal.map_name && goal.map_name.toLowerCase().includes(search)) ||
          goal.sequence_number?.toString().includes(search) ||
          this.formatCoordinate(goal.position_x).includes(search) ||
          this.formatCoordinate(goal.position_y).includes(search)
        )
      }

      return filtered
    },

    paginatedGoals() {
      const start = (this.currentPage - 1) * this.itemsPerPage
      const end = start + this.itemsPerPage
      return this.filteredGoals.slice(start, end)
    },

    totalPages() {
      return Math.ceil(this.filteredGoals.length / this.itemsPerPage)
    },

    filteredMissions() {
      if (this.missionFilter === 'all') return this.missions
      if (this.missionFilter === 'recent') {
        const oneWeekAgo = new Date()
        oneWeekAgo.setDate(oneWeekAgo.getDate() - 7)
        return this.missions.filter(mission => {
          const createdDate = new Date(mission.created_at)
          return createdDate >= oneWeekAgo
        })
      }
      if (this.missionFilter === 'with-goals') {
        return this.missions.filter(mission => mission.goal_count > 0)
      }
      return this.missions
    },

    missionProgress() {
      if (this.missionState.totalSteps === 0) return 0
      return Math.round((this.missionState.completedGoals / this.missionState.totalSteps) * 100)
    },

    canSaveMission() {
      return this.currentMission.name.trim() &&
        this.currentMission.category_id &&
        this.currentMission.goals.length > 0
    },

    canStartMission() {
      return this.isConnected &&
        this.currentMission.goals.length > 0 &&
        !this.missionState.isActive
    },

    canCreateMission() {
      return this.newMission.name.trim() && this.newMission.category_id
    },

    canSaveCategory() {
      return this.newCategory.name.trim()
    },

    canRunAllMissions() {
      return this.isConnected &&
        this.filteredMissions.length > 0 &&
        !this.missionState.isActive &&
        !this.isRunningAllMissions
    }
  },

  watch: {
    isConnected(newVal) {
      if (newVal) {
        this.setupROSTopics()
      }
    }
  },

  mounted() {
    this.loadInitialData()
    if (this.isConnected) {
      this.setupROSTopics()
    }

    this.timeInterval = setInterval(() => {
      this.updateElapsedTime()
    }, 1000)
  },

  beforeUnmount() {
    this.stopMission()
    this.stopAllMissions()
    if (this.timeInterval) {
      clearInterval(this.timeInterval)
    }
    if (this.completionTimer) {
      clearTimeout(this.completionTimer)
    }
    if (this.batchCompletionCheck) {
      clearInterval(this.batchCompletionCheck)
    }
  },

  methods: {
    setupROSTopics() {
      if (!this.ros) {
        this.addLog('error', 'ROS connection not available')
        return
      }

      try {
        this.goalTopic = new ROSLIB.Topic({
          ros: this.ros,
          name: '/goal_pose',
          messageType: 'geometry_msgs/msg/PoseStamped'
        })

        this.goalStatusTopic = new ROSLIB.Topic({
          ros: this.ros,
          name: '/move_base/feedback',
          messageType: 'move_base_msgs/msg/MoveBaseActionFeedback'
        })

        this.goalStatusTopic.subscribe((message) => {
          this.handleGoalStatusUpdate(message)
        })

        this.addLog('success', 'ROS topics initialized')
      } catch (error) {
        this.addLog('error', `Failed to setup ROS topics: ${error.message}`)
      }
    },

    async loadInitialData() {
      this.isLoading = true
      try {
        await Promise.all([
          this.loadCategories(),
          this.loadGoalSets(),
          this.loadGoals(),
          this.loadMissions()
        ])
        this.addLog('success', 'All data loaded successfully')
      } catch (error) {
        this.addLog('error', `Failed to load data: ${error.message}`)
      } finally {
        this.isLoading = false
      }
    },

    async loadCategories() {
      try {
        const response = await fetch(`http://${window.location.hostname}:3000/api/categories`)
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`)
        }
        const result = await response.json()
        if (result.success) {
          this.categories = result.categories
          this.addLog('info', `Loaded ${this.categories.length} categories`)
        } else {
          throw new Error(result.error || 'Failed to load categories')
        }
      } catch (error) {
        console.error('Error loading categories:', error)
        this.addLog('error', `Failed to load categories: ${error.message}`)
        this.categories = [
          { id: 1, name: 'Default', icon: 'fas fa-folder', color: '#3b82f6', mission_count: 0 }
        ]
      }
    },

    async loadGoalSets() {
      try {
        const response = await fetch(`http://${window.location.hostname}:3000/api/goals/sets`)
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`)
        }

        const result = await response.json()
        if (result.success) {
          this.goalSets = result.goalSets
          this.addLog('info', `Loaded ${this.goalSets.length} goal sets`)
        } else {
          throw new Error(result.error || 'Failed to load goal sets')
        }
      } catch (error) {
        console.error('Error loading goal sets:', error)
        this.addLog('error', `Failed to load goal sets: ${error.message}`)
        this.goalSets = []
      }
    },

    onGoalSetChange() {
      if (this.selectedGoalSet) {
        const selectedSet = this.goalSets.find(set => set.id === this.selectedGoalSet)
        this.selectedGoalSetName = selectedSet?.set_name || `Set ${this.selectedGoalSet}`
        this.addLog('info', `Selected goal set: ${this.selectedGoalSetName}`)
      } else {
        this.selectedGoalSetName = ''
        this.addLog('info', 'Showing all goal sets')
      }

      this.currentPage = 1
    },

    clearGoalSetSelection() {
      this.selectedGoalSet = null
      this.selectedGoalSetName = ''
      this.currentPage = 1
      this.addLog('info', 'Cleared goal set filter')
    },

    async loadGoals() {
      this.isLoadingGoals = true
      try {
        const response = await fetch(`http://${window.location.hostname}:3000/api/goals/sets`)
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`)
        }

        const result = await response.json()

        if (result.success) {
          this.allGoals = []

          for (const goalSet of result.goalSets) {
            try {
              const setResponse = await fetch(`http://${window.location.hostname}:3000/api/goals/set/id/${goalSet.id}`)
              if (setResponse.ok) {
                const setResult = await setResponse.json()

                if (setResult.success && setResult.goals) {
                  setResult.goals.forEach(goal => {
                    const processedGoal = {
                      id: goal.id,
                      goal_set_id: goal.goal_set_id,
                      sequence_number: goal.sequence_number || 1,
                      position_x: this.parseCoordinate(goal.position_x),
                      position_y: this.parseCoordinate(goal.position_y),
                      orientation_z: this.parseCoordinate(goal.orientation_z),
                      orientation_w: this.parseCoordinate(goal.orientation_w),
                      tolerance_xy: this.parseCoordinate(goal.tolerance_xy),
                      tolerance_yaw: this.parseCoordinate(goal.tolerance_yaw),
                      created_at: goal.created_at,
                      set_name: goalSet.set_name,
                      map_name: setResult.goalSet?.map_name || 'Unknown'
                    }
                    this.allGoals.push(processedGoal)
                  })
                }
              }
            } catch (setError) {
              console.error(`Error loading goals for set ${goalSet.id}:`, setError)
              this.addLog('error', `Failed to load goals for set ${goalSet.set_name}`)
            }
          }

          this.addLog('info', `Loaded ${this.allGoals.length} goals from database`)
        } else {
          throw new Error(result.error || 'Failed to load goal sets')
        }
      } catch (error) {
        console.error('Error loading goals:', error)
        this.addLog('error', `Failed to load goals: ${error.message}`)
        this.allGoals = []
      } finally {
        this.isLoadingGoals = false
      }
    },

    parseCoordinate(value) {
      if (value === null || value === undefined) return 0
      const num = parseFloat(value)
      return isNaN(num) ? 0 : num
    },

    formatCoordinate(value) {
      const num = this.parseCoordinate(value)
      return num.toFixed(2)
    },

    async loadMissions() {
      try {
        const response = await fetch(`http://${window.location.hostname}:3000/api/missions`)
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`)
        }

        const result = await response.json()

        if (result.success) {
          this.missions = result.missions || []
          this.addLog('info', `Loaded ${this.missions.length} missions`)
        } else {
          throw new Error(result.error || 'Failed to load missions')
        }
      } catch (error) {
        console.error('Error loading missions:', error)
        this.addLog('error', `Failed to load missions: ${error.message}`)
        this.missions = []
      }
    },

    async loadMission(mission) {
      try {
        console.log('Loading mission with triggers:', mission.id)
        const response = await fetch(`http://${window.location.hostname}:3000/api/missions/with-triggers/${mission.id}`)
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`)
        }

        const result = await response.json()
        console.log('Mission loaded:', result)

        if (result.success) {
          this.selectedMission = mission
          this.currentMission = {
            id: mission.id,
            name: mission.name,
            description: mission.description || '',
            category_id: mission.category_id,
            goals: (result.mission.goals || []).map(goal => ({
              ...goal,
              position_x: this.parseCoordinate(goal.position_x),
              position_y: this.parseCoordinate(goal.position_y),
              next_goal_trigger: goal.next_goal_trigger || 'auto',
              wait_time: goal.wait_time || 5,
              sensor_type: goal.sensor_type || '',
              sensor_condition: goal.sensor_condition || '',
              timeout: goal.timeout || 60,
              retry_count: goal.retry_count || 0,
              on_failure: goal.on_failure || 'skip',
              showAdvanced: false
            }))
          }
          this.addLog('success', `Loaded mission: ${mission.name}`)
        }
      } catch (error) {
        console.error('Error loading mission:', error)
        this.addLog('error', `Failed to load mission: ${error.message}`)
      }
    },

    refreshData() {
      this.loadInitialData()
    },

    showNewCategoryModal() {
      this.newCategory = {
        id: null,
        name: '',
        description: '',
        icon: 'fas fa-folder',
        color: '#3b82f6'
      }
      this.editingCategory = false
      this.showCategoryModal = true
    },

    editCategory(category) {
      this.newCategory = { ...category }
      this.editingCategory = true
      this.showCategoryModal = true
    },

    async saveCategory() {
      if (!this.canSaveCategory) {
        this.addLog('warning', 'Please enter a category name')
        return
      }

      try {
        const url = this.editingCategory
          ? `http://${window.location.hostname}:3000/api/categories/${this.newCategory.id}`
          : `http://${window.location.hostname}:3000/api/categories`

        const method = this.editingCategory ? 'PUT' : 'POST'

        const response = await fetch(url, {
          method,
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(this.newCategory)
        })

        if (!response.ok) {
          const error = await response.json()
          throw new Error(error.error || 'Failed to save category')
        }

        const result = await response.json()

        if (result.success) {
          this.addLog('success', `Category "${this.newCategory.name}" ${this.editingCategory ? 'updated' : 'created'}`)
          await this.loadCategories()
          this.closeModal()
        }
      } catch (error) {
        console.error('Error saving category:', error)
        this.addLog('error', `Failed to save category: ${error.message}`)
      }
    },

    async deleteCategory(categoryId) {
      if (!confirm('Are you sure you want to delete this category? All missions in this category will be moved to Default category.')) {
        return
      }

      try {
        const response = await fetch(`http://${window.location.hostname}:3000/api/categories/${categoryId}`, {
          method: 'DELETE'
        })

        if (!response.ok) {
          const error = await response.json()
          throw new Error(error.error || 'Failed to delete category')
        }

        const result = await response.json()

        if (result.success) {
          this.addLog('success', 'Category deleted successfully')
          await this.loadCategories()
          if (this.selectedCategory === categoryId) {
            this.selectedCategory = null
          }
        }
      } catch (error) {
        console.error('Error deleting category:', error)
        this.addLog('error', `Failed to delete category: ${error.message}`)
      }
    },

    selectCategory(categoryId) {
      this.selectedCategory = categoryId
    },

    getCategoryIcon(categoryId) {
      const category = this.categories.find(c => c.id === categoryId)
      return category?.icon || 'fas fa-folder'
    },

    getCategoryName(categoryId) {
      const category = this.categories.find(c => c.id === categoryId)
      return category?.name || 'Uncategorized'
    },

    previewGoal(goal) {
      this.addLog('info', `Previewing goal: X=${this.formatCoordinate(goal.position_x)}, Y=${this.formatCoordinate(goal.position_y)}`)
      this.$emit('goal-preview', goal)
    },

    isGoalInMission(goalId) {
      return this.currentMission.goals.some(g => g.id === goalId)
    },

    addGoalToCurrentMission(goal) {
      if (!this.isGoalInMission(goal.id)) {
        const goalCopy = {
          ...goal,
          position_x: this.parseCoordinate(goal.position_x),
          position_y: this.parseCoordinate(goal.position_y),
          next_goal_trigger: 'auto',
          wait_time: 5,
          sensor_type: '',
          sensor_condition: '',
          timeout: 60,
          retry_count: 0,
          on_failure: 'skip',
          showAdvanced: false
        }
        this.currentMission.goals.push(goalCopy)
        this.addLog('success', `Added goal to mission: X=${this.formatCoordinate(goal.position_x)}, Y=${this.formatCoordinate(goal.position_y)}`)
      } else {
        this.addLog('warning', 'Goal already in mission')
      }
    },

    removeGoalFromMission(index) {
      const removedGoal = this.currentMission.goals[index]
      this.currentMission.goals.splice(index, 1)
      this.addLog('info', `Removed goal from mission`)
    },

    moveGoalUp(index) {
      if (index > 0) {
        const goal = this.currentMission.goals[index]
        this.currentMission.goals.splice(index, 1)
        this.currentMission.goals.splice(index - 1, 0, goal)
      }
    },

    moveGoalDown(index) {
      if (index < this.currentMission.goals.length - 1) {
        const goal = this.currentMission.goals[index]
        this.currentMission.goals.splice(index, 1)
        this.currentMission.goals.splice(index + 1, 0, goal)
      }
    },

    clearMissionSequence() {
      if (confirm('Clear all goals from mission sequence?')) {
        this.currentMission.goals = []
        this.addLog('info', 'Cleared mission sequence')
      }
    },

    optimizeSequence() {
      if (this.currentMission.goals.length < 2) {
        this.addLog('warning', 'Need at least 2 goals to optimize')
        return
      }

      const optimized = [this.currentMission.goals[0]]
      const remaining = [...this.currentMission.goals.slice(1)]

      while (remaining.length > 0) {
        const lastGoal = optimized[optimized.length - 1]
        let nearestIndex = 0
        let nearestDistance = this.calculateDistance(lastGoal, remaining[0])

        for (let i = 1; i < remaining.length; i++) {
          const distance = this.calculateDistance(lastGoal, remaining[i])
          if (distance < nearestDistance) {
            nearestDistance = distance
            nearestIndex = i
          }
        }

        optimized.push(remaining[nearestIndex])
        remaining.splice(nearestIndex, 1)
      }

      this.currentMission.goals = optimized
      this.addLog('success', 'Optimized mission sequence for shortest path')
    },

    calculateDistance(goal1, goal2) {
      const dx = this.parseCoordinate(goal2.position_x) - this.parseCoordinate(goal1.position_x)
      const dy = this.parseCoordinate(goal2.position_y) - this.parseCoordinate(goal1.position_y)
      return Math.sqrt(dx * dx + dy * dy)
    },

    calculateTotalDistance() {
      if (this.currentMission.goals.length < 2) return 0

      let total = 0
      for (let i = 1; i < this.currentMission.goals.length; i++) {
        total += this.calculateDistance(this.currentMission.goals[i - 1], this.currentMission.goals[i])
      }
      return total
    },

    estimateMissionTime() {
      const distance = this.calculateTotalDistance()
      const avgSpeed = 0.5
      const timeSeconds = distance / avgSpeed
      const setupTime = 5
      const totalTime = timeSeconds + (this.currentMission.goals.length * setupTime)

      if (totalTime < 60) {
        return `${Math.round(totalTime)}s`
      } else if (totalTime < 3600) {
        return `${Math.round(totalTime / 60)}m`
      } else {
        const hours = Math.floor(totalTime / 3600)
        const minutes = Math.round((totalTime % 3600) / 60)
        return `${hours}h ${minutes}m`
      }
    },

    showNewMissionModal() {
      this.newMission = {
        name: '',
        description: '',
        category_id: this.selectedCategory || this.categories[0]?.id || ''
      }
      this.showMissionModal = true
    },

    async createNewMission() {
      console.log('createNewMission called', this.newMission)

      if (!this.canCreateMission) {
        console.log('Validation failed')
        this.addLog('warning', 'Please enter mission name and select category')
        return
      }

      try {
        const missionData = {
          name: this.newMission.name.trim(),
          description: this.newMission.description?.trim() || '',
          category_id: parseInt(this.newMission.category_id),
          goals: [] // Kosong untuk mission baru
        }

        console.log('Sending mission data:', missionData)

        // Coba endpoint tanpa goals terlebih dahulu
        let response = await fetch(`http://${window.location.hostname}:3000/api/missions`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(missionData)
        })

        // Jika gagal dengan endpoint pertama, coba endpoint kedua
        if (!response.ok) {
          response = await fetch(`http://${window.location.hostname}:3000/api/missions/with-triggers`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json'
            },
            body: JSON.stringify(missionData)
          })
        }

        const result = await response.json()
        console.log('API Response:', result)

        if (!response.ok) {
          throw new Error(result.error || `HTTP ${response.status}`)
        }

        if (result.success) {
          this.addLog('success', `Mission "${missionData.name}" created successfully`)
          await this.loadMissions()
          this.closeModal()

          // Load mission yang baru dibuat
          if (result.mission) {
            await this.loadMission(result.mission)
          }
        } else {
          throw new Error(result.error || 'Unknown error')
        }
      } catch (error) {
        console.error('Full error creating mission:', error)
        console.error('Stack trace:', error.stack)
        this.addLog('error', `Failed to create mission: ${error.message}`)

        // Tampilkan user-friendly message
        if (error.message.includes('at least one goal')) {
          this.addLog('warning', 'Created mission with empty goals list. You can add goals later.')

          // Fallback: Buat mission dengan dummy goal jika diperlukan
          await this.createMissionWithDummyGoal()
        }
      }
    },

    // Method fallback untuk membuat mission dengan dummy goal
    async createMissionWithDummyGoal() {
      try {
        const missionData = {
          name: this.newMission.name.trim(),
          description: this.newMission.description?.trim() || '',
          category_id: parseInt(this.newMission.category_id),
          goals: [{
            id: 0, // Temporary ID
            goal_set_id: 1, // Default goal set
            sequence_number: 1,
            position_x: 0.0,
            position_y: 0.0,
            orientation_z: 0.0,
            orientation_w: 1.0,
            tolerance_xy: 0.5,
            tolerance_yaw: 0.1,
            next_goal_trigger: 'auto',
            wait_time: 5,
            sensor_type: '',
            sensor_condition: '',
            timeout: 60,
            retry_count: 0,
            on_failure: 'skip'
          }]
        }

        const response = await fetch(`http://${window.location.hostname}:3000/api/missions/with-triggers`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(missionData)
        })

        const result = await response.json()

        if (result.success) {
          this.addLog('success', `Mission "${missionData.name}" created with placeholder goal`)
          await this.loadMissions()
          this.closeModal()

          if (result.mission) {
            await this.loadMission(result.mission)
          }
        }
      } catch (fallbackError) {
        console.error('Fallback creation failed:', fallbackError)
      }
    },

    async saveMission() {
      if (!this.canSaveMission) {
        this.addLog('warning', 'Please enter mission name, select category, and add at least one goal')
        return
      }

      try {
        // Prepare mission data with triggers
        const missionData = {
          name: this.currentMission.name,
          description: this.currentMission.description,
          category_id: this.currentMission.category_id,
          goals: this.currentMission.goals.map((goal, index) => ({
            id: goal.id,
            goal_set_id: goal.goal_set_id,
            sequence_number: goal.sequence_number || (index + 1),
            position_x: this.parseCoordinate(goal.position_x),
            position_y: this.parseCoordinate(goal.position_y),
            orientation_z: this.parseCoordinate(goal.orientation_z),
            orientation_w: this.parseCoordinate(goal.orientation_w || 1.0),
            tolerance_xy: this.parseCoordinate(goal.tolerance_xy),
            tolerance_yaw: this.parseCoordinate(goal.tolerance_yaw),
            next_goal_trigger: goal.next_goal_trigger || 'auto',
            wait_time: goal.wait_time || 5,
            sensor_type: goal.sensor_type || '',
            sensor_condition: goal.sensor_condition || '',
            timeout: goal.timeout || 60,
            retry_count: goal.retry_count || 0,
            on_failure: goal.on_failure || 'skip'
          }))
        }

        const url = this.currentMission.id
          ? `http://${window.location.hostname}:3000/api/missions/with-triggers/${this.currentMission.id}`
          : `http://${window.location.hostname}:3000/api/missions/with-triggers`

        const method = this.currentMission.id ? 'PUT' : 'POST'

        console.log('Saving mission:', { url, method, missionData })

        const response = await fetch(url, {
          method,
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(missionData)
        })

        const result = await response.json()
        console.log('Save mission response:', result)

        if (!response.ok) {
          throw new Error(result.error || 'Failed to save mission')
        }

        if (result.success) {
          const action = this.currentMission.id ? 'updated' : 'saved'
          this.addLog('success', `Mission "${missionData.name}" ${action} successfully`)

          // Reload missions
          await this.loadMissions()

          // Update current mission data with the returned data
          if (result.mission) {
            this.currentMission = {
              id: result.mission.id,
              name: result.mission.name,
              description: result.mission.description || '',
              category_id: result.mission.category_id,
              goals: (result.mission.goals || []).map(goal => ({
                ...goal,
                position_x: this.parseCoordinate(goal.position_x),
                position_y: this.parseCoordinate(goal.position_y),
                next_goal_trigger: goal.next_goal_trigger || 'auto',
                wait_time: goal.wait_time || 5,
                sensor_type: goal.sensor_type || '',
                sensor_condition: goal.sensor_condition || '',
                timeout: goal.timeout || 60,
                retry_count: goal.retry_count || 0,
                on_failure: goal.on_failure || 'skip',
                showAdvanced: false
              }))
            }
            this.selectedMission = result.mission
          }
        }
      } catch (error) {
        console.error('Error saving mission:', error)
        this.addLog('error', `Failed to save mission: ${error.message}`)
      }
    },

    async deleteMission(missionId) {
      if (!confirm('Are you sure you want to delete this mission?')) return

      try {
        const response = await fetch(`http://${window.location.hostname}:3000/api/missions/${missionId}`, {
          method: 'DELETE'
        })

        if (!response.ok) {
          const error = await response.json()
          throw new Error(error.error || 'Failed to delete mission')
        }

        const result = await response.json()

        if (result.success) {
          this.addLog('success', 'Mission deleted successfully')
          await this.loadMissions()

          if (this.selectedMission?.id === missionId) {
            this.selectedMission = null
            this.currentMission = {
              id: null,
              name: '',
              description: '',
              category_id: '',
              goals: []
            }
          }
        }
      } catch (error) {
        console.error('Error deleting mission:', error)
        this.addLog('error', `Failed to delete mission: ${error.message}`)
      }
    },

    async executeMission(mission) {
      if (!this.isConnected) {
        this.addLog('error', 'Cannot execute mission: ROS not connected')
        return
      }

      if (!this.selectedMission || this.selectedMission.id !== mission.id) {
        await this.loadMission(mission)
      }

      if (this.currentMission.goals.length === 0) {
        this.addLog('error', 'Cannot execute mission: No goals in mission')
        return
      }

      try {
        await fetch(`http://${window.location.hostname}:3000/api/missions/${mission.id}/execute`, {
          method: 'PATCH',
          headers: { 'Content-Type': 'application/json' }
        })
      } catch (error) {
        console.error('Failed to update mission timestamp:', error)
      }

      this.startMissionExecution()
    },

    // ====== TRIGGER-BASED MISSION EXECUTION ======

    startMissionExecution() {
      if (this.currentMission.goals.length === 0) {
        this.addLog('error', 'Cannot start mission: No goals in mission')
        return
      }

      if (this.missionState.isActive) {
        this.addLog('warning', 'Mission is already active')
        return
      }

      // Initialize trigger properties untuk setiap goal jika belum ada
      this.currentMission.goals.forEach(goal => {
        if (!goal.next_goal_trigger) {
          goal.next_goal_trigger = 'auto'
        }
        if (!goal.showAdvanced) {
          goal.showAdvanced = false
        }
      })

      this.missionState = {
        status: 'executing',
        isActive: true,
        isPaused: false,
        currentStep: 1,
        totalSteps: this.currentMission.goals.length,
        completedGoals: 0,
        startedAt: new Date().toLocaleTimeString(),
        elapsedTime: '00:00'
      }

      this.goalStartTime = Date.now()

      this.addLog('info', `🚀 Mission "${this.currentMission.name}" started`)
      this.addLog('info', `📊 Executing ${this.currentMission.goals.length} goals with triggers`)

      this.executeCurrentGoalWithTrigger()
    },

    async executeCurrentGoalWithTrigger() {
      if (!this.missionState.isActive || this.missionState.isPaused) return

      if (this.missionState.currentStep > this.currentMission.goals.length) {
        this.completeMission()
        return
      }

      const goal = this.currentMission.goals[this.missionState.currentStep - 1]
      this.currentGoal = goal
      this.goalStartTime = Date.now()
      this.currentGoalTime = 0

      this.addLog('info', `🎯 Step ${this.missionState.currentStep}: ${goal.set_name || `Goal ${goal.sequence_number}`}`)

      try {
        await this.sendGoalToRobot(goal)
        this.addLog('success', `✅ Goal sent to robot`)

        this.startGoalMonitoring()

      } catch (error) {
        this.addLog('error', `❌ Failed to execute goal: ${error.message}`)
        this.handleGoalFailure()
      }
    },

    async sendGoalToRobot(goal) {
      if (!this.goalTopic || !this.isConnected) {
        throw new Error('ROS not connected')
      }

      return new Promise((resolve, reject) => {
        try {
          const goalMessage = new ROSLIB.Message({
            header: {
              stamp: { sec: 0, nanosec: 0 },
              frame_id: 'map'
            },
            pose: {
              position: {
                x: this.parseCoordinate(goal.position_x),
                y: this.parseCoordinate(goal.position_y),
                z: 0.0
              },
              orientation: {
                x: 0.0,
                y: 0.0,
                z: this.parseCoordinate(goal.orientation_z),
                w: this.parseCoordinate(goal.orientation_w || 1.0)
              }
            }
          })

          this.goalTopic.publish(goalMessage)
          resolve()
        } catch (error) {
          reject(error)
        }
      })
    },

    startGoalMonitoring() {
      if (this.completionTimer) {
        clearTimeout(this.completionTimer)
      }

      const completionTime = 3000 + Math.random() * 2000
      this.completionTimer = setTimeout(() => {
        this.handleGoalCompletion()
      }, completionTime)
    },

    stopGoalMonitoring() {
      if (this.completionTimer) {
        clearTimeout(this.completionTimer)
        this.completionTimer = null
      }
    },

    handleGoalStatusUpdate(message) {
      console.log('Goal status update:', message)
    },

    async handleGoalCompletion() {
      if (!this.missionState.isActive) return

      const currentGoal = this.currentMission.goals[this.missionState.currentStep - 1]

      // Check trigger sebelum lanjut ke goal berikutnya
      const shouldProceed = await this.checkGoalTrigger(currentGoal)

      if (shouldProceed) {
        this.addLog('success', `✅ Step ${this.missionState.currentStep} completed`)
        this.missionState.completedGoals++
        this.missionState.currentStep++

        this.stopGoalMonitoring()
        this.clearTriggerState()

        if (this.missionState.currentStep > this.currentMission.goals.length) {
          this.completeMission()
        } else {
          setTimeout(() => {
            this.executeCurrentGoalWithTrigger()
          }, 1000)
        }
      } else {
        this.handleTriggerFailure(currentGoal)
      }
    },

    async checkGoalTrigger(goal) {
      if (!goal.next_goal_trigger || goal.next_goal_trigger === 'auto') {
        return true
      }

      this.currentTriggerState = {
        type: goal.next_goal_trigger,
        waiting: true,
        timer: null,
        manualConfirmation: false,
        sensorValue: null,
        timeoutTimer: null
      }

      this.addLog('info', `⚙️ Waiting for trigger: ${goal.next_goal_trigger}`)

      switch (goal.next_goal_trigger) {
        case 'timer':
          return await this.waitForTimer(goal.wait_time || 5)

        case 'manual':
          return await this.waitForManualConfirmation(goal)

        case 'sensor':
          return await this.waitForSensorCondition(goal)

        default:
          return true
      }
    },

    async waitForTimer(seconds) {
      return new Promise((resolve) => {
        this.addLog('info', `⏱️ Waiting ${seconds} seconds...`)

        let remaining = seconds
        const countdown = setInterval(() => {
          remaining--
          if (remaining <= 0) {
            clearInterval(countdown)
            this.addLog('success', `✅ Timer completed`)
            resolve(true)
          }
        }, 1000)

        this.currentTriggerState.timer = setTimeout(() => {
          clearInterval(countdown)
          resolve(true)
        }, seconds * 1000)
      })
    },

    async waitForManualConfirmation(goal) {
      return new Promise((resolve) => {
        this.addLog('warning', `⏸️ Waiting for manual confirmation...`)
        this.currentTriggerState.manualConfirmation = true

        const confirmAction = () => {
          this.addLog('success', `✅ Manual confirmation received`)
          this.currentTriggerState.manualConfirmation = false
          resolve(true)
        }

        this.manualConfirmCallback = confirmAction

        if (goal.timeout) {
          this.currentTriggerState.timeoutTimer = setTimeout(() => {
            this.addLog('warning', `⏰ Manual confirmation timeout`)
            this.currentTriggerState.manualConfirmation = false
            resolve(false)
          }, goal.timeout * 1000)
        }
      })
    },

    async waitForSensorCondition(goal) {
      return new Promise((resolve) => {
        this.addLog('info', `📡 Waiting for sensor condition: ${goal.sensor_condition}`)

        const checkCondition = () => {
          setTimeout(() => {
            this.addLog('success', `✅ Sensor condition met`)
            resolve(true)
          }, 2000)
        }

        checkCondition()

        if (goal.timeout) {
          this.currentTriggerState.timeoutTimer = setTimeout(() => {
            this.addLog('warning', `⏰ Sensor condition timeout`)
            resolve(false)
          }, goal.timeout * 1000)
        }
      })
    },

    clearTriggerState() {
      if (this.currentTriggerState.timer) {
        clearTimeout(this.currentTriggerState.timer)
      }
      if (this.currentTriggerState.timeoutTimer) {
        clearTimeout(this.currentTriggerState.timeoutTimer)
      }
      this.currentTriggerState = {
        type: 'auto',
        waiting: false,
        timer: null,
        manualConfirmation: false,
        sensorValue: null,
        timeoutTimer: null
      }
    },

    confirmManualTrigger() {
      if (this.manualConfirmCallback) {
        this.manualConfirmCallback()
        this.manualConfirmCallback = null
      }
    },

    handleTriggerFailure(goal) {
      const action = goal.on_failure || 'skip'

      switch (action) {
        case 'skip':
          this.addLog('warning', `⏭️ Trigger failed, skipping to next goal`)
          this.missionState.currentStep++
          this.executeCurrentGoalWithTrigger()
          break

        case 'retry':
          if (goal.retry_count && goal.retry_count > 0) {
            goal.retry_count--
            this.addLog('warning', `🔄 Trigger failed, retrying...`)
            this.executeCurrentGoalWithTrigger()
          } else {
            this.addLog('error', `❌ Max retries reached, skipping`)
            this.missionState.currentStep++
            this.executeCurrentGoalWithTrigger()
          }
          break

        case 'stop':
          this.addLog('error', `❌ Trigger failed, stopping mission`)
          this.stopMission()
          break

        case 'pause':
          this.addLog('warning', `⏸️ Trigger failed, pausing mission`)
          this.pauseMission()
          break
      }
    },

    handleGoalFailure() {
      if (!this.missionState.isActive) return

      this.addLog('error', `❌ Step ${this.missionState.currentStep} failed`)
      this.stopGoalMonitoring()

      setTimeout(() => {
        this.addLog('warning', `🔄 Retrying Step ${this.missionState.currentStep}`)
        this.executeCurrentGoalWithTrigger()
      }, 3000)
    },

    pauseMission() {
      if (!this.missionState.isActive || this.missionState.isPaused) return

      this.missionState.isPaused = true
      this.missionState.status = 'paused'
      this.stopGoalMonitoring()
      this.clearTriggerState()
      this.addLog('warning', '⏸️ Mission paused')
    },

    resumeMission() {
      if (!this.missionState.isActive || !this.missionState.isPaused) return

      this.missionState.isPaused = false
      this.missionState.status = 'executing'
      this.addLog('info', '▶️ Mission resumed')

      this.executeCurrentGoalWithTrigger()
    },

    stopMission() {
      if (!this.missionState.isActive && !this.isRunningAllMissions) return

      this.missionState.isActive = false
      this.missionState.isPaused = false
      this.missionState.status = 'stopped'
      this.stopGoalMonitoring()
      this.clearTriggerState()
      this.currentGoal = null
      this.goalStartTime = null

      if (this.isRunningAllMissions) {
        this.stopAllMissions()
      } else {
        this.addLog('warning', '⏹️ Mission stopped')
      }
    },

    cancelMission() {
      if (!this.missionState.isActive) return

      if (confirm('Cancel mission and reset progress?')) {
        this.stopMission()
        this.missionState.completedGoals = 0
        this.missionState.currentStep = 1
        this.addLog('warning', 'Mission canceled and reset')
      }
    },

    skipToNextGoal() {
      if (!this.missionState.isActive) return

      this.addLog('info', `⏭️ Skipping Step ${this.missionState.currentStep}`)
      this.stopGoalMonitoring()
      this.clearTriggerState()
      this.missionState.currentStep++

      if (this.missionState.currentStep > this.currentMission.goals.length) {
        this.completeMission()
      } else {
        this.executeCurrentGoalWithTrigger()
      }
    },

    retryCurrentGoal() {
      if (!this.missionState.isActive) return

      this.addLog('info', `🔄 Retrying Step ${this.missionState.currentStep}`)
      this.stopGoalMonitoring()
      this.clearTriggerState()
      this.executeCurrentGoalWithTrigger()
    },

    completeMission() {
      this.missionState.isActive = false
      this.missionState.isPaused = false
      this.missionState.status = 'completed'
      this.currentGoal = null
      this.goalStartTime = null
      this.stopGoalMonitoring()
      this.clearTriggerState()

      this.addLog('success', `🎉 Mission "${this.currentMission.name}" completed!`)
      this.addLog('success', `📊 ${this.missionState.completedGoals} of ${this.missionState.totalSteps} goals completed`)
    },

    // ====== RUN ALL MISSIONS FUNCTIONALITY ======

    async runAllMissions() {
      if (!this.isConnected) {
        this.addLog('error', 'Cannot run missions: ROS not connected')
        return
      }

      if (this.filteredMissions.length === 0) {
        this.addLog('error', 'No missions available to run')
        return
      }

      const missionsWithGoals = this.filteredMissions.filter(m => m.goal_count > 0)

      if (missionsWithGoals.length === 0) {
        this.addLog('error', 'No missions with goals available to run')
        return
      }

      if (!confirm(`Run all ${missionsWithGoals.length} missions? This will execute each mission sequentially.`)) {
        return
      }

      this.addLog('info', `🚀 Starting to run all ${missionsWithGoals.length} missions`)

      this.isRunningAllMissions = true
      this.batchMissions = [...missionsWithGoals]
      this.currentBatchMissionIndex = 0
      this.missionState.status = 'batch-executing'

      await this.executeNextMissionInBatch()
    },

    async executeNextMissionInBatch() {
      if (!this.isRunningAllMissions || this.currentBatchMissionIndex >= this.batchMissions.length) {
        this.completeAllMissions()
        return
      }

      const mission = this.batchMissions[this.currentBatchMissionIndex]
      const missionNumber = this.currentBatchMissionIndex + 1
      const totalMissions = this.batchMissions.length

      this.addLog('info', `📋 Mission ${missionNumber}/${totalMissions}: ${mission.name}`)

      try {
        await this.loadMission(mission)

        this.startMissionExecution()

        this.monitorBatchMissionCompletion()

      } catch (error) {
        this.addLog('error', `Failed to execute mission ${mission.name}: ${error.message}`)
        this.currentBatchMissionIndex++
        setTimeout(() => this.executeNextMissionInBatch(), 1000)
      }
    },

    monitorBatchMissionCompletion() {
      if (this.batchCompletionCheck) {
        clearInterval(this.batchCompletionCheck)
      }

      this.batchCompletionCheck = setInterval(() => {
        if (!this.missionState.isActive && !this.missionState.isPaused) {
          clearInterval(this.batchCompletionCheck)
          this.batchCompletionCheck = null

          const mission = this.batchMissions[this.currentBatchMissionIndex]
          const missionNumber = this.currentBatchMissionIndex + 1

          if (this.missionState.status === 'completed') {
            this.addLog('success', `✅ Mission ${missionNumber}: ${mission.name} completed successfully`)
          } else {
            this.addLog('warning', `⚠️ Mission ${missionNumber}: ${mission.name} ended with status: ${this.missionState.status}`)
          }

          this.currentBatchMissionIndex++

          setTimeout(() => {
            this.executeNextMissionInBatch()
          }, 2000)
        }
      }, 1000)
    },

    stopAllMissions() {
      this.isRunningAllMissions = false
      this.batchMissions = []
      this.currentBatchMissionIndex = 0

      if (this.batchCompletionCheck) {
        clearInterval(this.batchCompletionCheck)
        this.batchCompletionCheck = null
      }

      this.addLog('warning', '⏹️ All missions execution stopped')
      this.missionState.status = 'idle'
    },

    completeAllMissions() {
      this.isRunningAllMissions = false
      this.batchMissions = []
      this.currentBatchMissionIndex = 0

      if (this.batchCompletionCheck) {
        clearInterval(this.batchCompletionCheck)
        this.batchCompletionCheck = null
      }

      this.addLog('success', `🎉 All missions completed!`)
      this.missionState.status = 'idle'
    },

    updateElapsedTime() {
      if (this.goalStartTime) {
        this.currentGoalTime = Math.floor((Date.now() - this.goalStartTime) / 1000)
      }
    },

    addLog(type, message) {
      const time = new Date().toLocaleTimeString()
      this.missionLogs.unshift({
        time,
        type,
        message
      })

      if (this.missionLogs.length > 100) {
        this.missionLogs = this.missionLogs.slice(0, 100)
      }
    },

    clearLogs() {
      this.missionLogs = []
      this.addLog('info', 'Logs cleared')
    },

    getLogIcon(type) {
      switch (type) {
        case 'success': return 'fas fa-check-circle'
        case 'error': return 'fas fa-exclamation-circle'
        case 'warning': return 'fas fa-exclamation-triangle'
        case 'info': return 'fas fa-info-circle'
        default: return 'fas fa-info-circle'
      }
    },

    formatDate(dateString) {
      if (!dateString) return 'N/A'
      try {
        const date = new Date(dateString)
        return date.toLocaleDateString('en-US', {
          month: 'short',
          day: 'numeric',
          year: 'numeric'
        })
      } catch {
        return 'Invalid Date'
      }
    },

    closeModal() {
      this.showMissionModal = false
      this.showCategoryModal = false
      this.editingCategory = false
    },

    nextPage() {
      if (this.currentPage < this.totalPages) {
        this.currentPage++
      }
    },

    prevPage() {
      if (this.currentPage > 1) {
        this.currentPage--
      }
    }
  }
}
</script>

<style scoped>
/* Reset and Base Styles */
* {
  box-sizing: border-box;
}

.mission-control {
  width: 100%;
  display: flex;
  flex-direction: column;
  gap: 16px;
  padding: 12px;
  background: var(--background-color);
  overflow: hidden;
}

/* Header Section */
.header-section {
  background: linear-gradient(135deg, rgba(30, 41, 59, 0.9), rgba(15, 23, 42, 0.95));
  border-radius: 12px;
  padding: 16px;
  border: 1px solid var(--border-color);
  display: flex;
  flex-direction: column;
  gap: 16px;
  backdrop-filter: blur(10px);
}

.header-content {
  display: flex;
  align-items: center;
  gap: 12px;
  flex-wrap: wrap;
}

.header-icon {
  width: 48px;
  height: 48px;
  background: linear-gradient(135deg, #f59e0b, #f97316);
  border-radius: 10px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 20px;
  color: white;
  flex-shrink: 0;
}

.header-text {
  flex: 1;
  min-width: 200px;
}

.header-text h1 {
  margin: 0 0 4px 0;
  font-size: 20px;
  font-weight: 700;
  background: linear-gradient(135deg, #f59e0b, #f97316);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.subtitle {
  margin: 0;
  font-size: 12px;
  color: var(--text-muted);
  line-height: 1.4;
}

.header-actions {
  display: flex;
  gap: 8px;
  flex-wrap: wrap;
}

.header-btn {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 8px 14px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  color: var(--text-secondary);
  border-radius: 8px;
  font-weight: 500;
  font-size: 12px;
  transition: all 0.3s ease;
  min-height: 36px;
  flex: 1;
  min-width: 120px;
  justify-content: center;
}

.header-btn:hover:not(:disabled) {
  background-color: rgba(255, 255, 255, 0.1);
  color: var(--text-primary);
}

.header-btn.primary {
  background: linear-gradient(135deg, #3b82f6, #8b5cf6);
  color: white;
  border: none;
}

.header-btn.primary:hover:not(:disabled) {
  box-shadow: 0 6px 20px rgba(59, 130, 246, 0.4);
}

.header-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

/* Main Grid Layout */
.main-grid {
  display: grid;
  grid-template-columns: repeat(1, 1fr);
  gap: 16px;
  flex: 1;
  min-height: 0;
  overflow-y: auto;
  padding-bottom: 20px;
}

.column {
  display: flex;
  flex-direction: column;
  gap: 16px;
  min-height: 0;
}

/* Card Styles */
.card {
  background-color: var(--card-bg);
  border: 1px solid var(--border-color);
  border-radius: 12px;
  padding: 16px;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  height: fit-content;
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
  padding-bottom: 8px;
  border-bottom: 1px solid var(--border-color);
  flex-wrap: wrap;
  gap: 8px;
}

.header-left {
  display: flex;
  align-items: center;
  gap: 8px;
  min-width: 0;
}

.header-left h3 {
  margin: 0;
  font-size: 14px;
  font-weight: 600;
  color: var(--text-primary);
  display: flex;
  align-items: center;
  gap: 6px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.count-badge {
  background: var(--accent-blue);
  color: white;
  padding: 2px 6px;
  border-radius: 10px;
  font-size: 10px;
  font-weight: 600;
  min-width: 20px;
  text-align: center;
  flex-shrink: 0;
}

.icon-btn {
  width: 28px;
  height: 28px;
  border: none;
  background: none;
  color: var(--text-secondary);
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: all 0.3s ease;
  flex-shrink: 0;
}

.icon-btn:hover {
  background-color: rgba(255, 255, 255, 0.1);
  color: var(--text-primary);
}

.icon-btn.danger {
  color: var(--accent-red);
}

.icon-btn.danger:hover {
  background-color: rgba(239, 68, 68, 0.1);
}

.action-btn {
  display: inline-flex;
  align-items: center;
  gap: 4px;
  padding: 5px 10px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  color: var(--text-secondary);
  border-radius: 6px;
  font-size: 11px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.3s ease;
  white-space: nowrap;
  min-height: 28px;
  flex-shrink: 0;
  overflow: hidden;
  text-overflow: ellipsis;
}

.action-btn:hover:not(:disabled) {
  background-color: rgba(255, 255, 255, 0.1);
  color: var(--text-primary);
}

.action-btn.small {
  padding: 4px 8px;
  font-size: 10px;
  min-height: 24px;
}

.action-btn.primary {
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  color: white;
  border: none;
}

.action-btn.success {
  background-color: rgba(16, 185, 129, 0.1);
  border-color: rgba(16, 185, 129, 0.3);
  color: var(--accent-green);
}

.action-btn.warning {
  background-color: rgba(245, 158, 11, 0.1);
  border-color: rgba(245, 158, 11, 0.3);
  color: var(--accent-yellow);
}

.action-btn.danger {
  background-color: rgba(239, 68, 68, 0.1);
  border-color: rgba(239, 68, 68, 0.3);
  color: var(--accent-red);
}

.action-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.btn-text {
  display: inline;
}

/* Categories Panel */
.categories-list {
  flex: 1;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 6px;
  min-height: 150px;
  max-height: 300px;
}

.category-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  transition: all 0.3s ease;
  cursor: pointer;
  min-height: 56px;
  overflow: hidden;
}

.category-item:hover {
  background-color: rgba(255, 255, 255, 0.05);
  border-color: var(--accent-blue);
}

.category-item.active {
  background-color: rgba(59, 130, 246, 0.1);
  border-color: var(--accent-blue);
}

.category-info {
  flex: 1;
  min-width: 0;
  overflow: hidden;
}

.category-name {
  display: flex;
  align-items: center;
  gap: 6px;
  font-weight: 500;
  color: var(--text-primary);
  margin-bottom: 2px;
  font-size: 13px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.category-stats {
  display: flex;
  gap: 12px;
  font-size: 11px;
  color: var(--text-muted);
}

.mission-count {
  display: flex;
  align-items: center;
  gap: 4px;
}

.category-actions {
  display: flex;
  gap: 4px;
  flex-shrink: 0;
}

/* Goals Panel */
.filter-controls {
  display: flex;
  gap: 8px;
  align-items: center;
  flex-wrap: wrap;
}

.filter-select {
  padding: 6px 10px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  color: var(--text-primary);
  border-radius: 6px;
  font-size: 11px;
  min-width: 120px;
}

.search-box {
  position: relative;
  display: flex;
  align-items: center;
  flex: 1;
  min-width: 150px;
}

.search-box .fas {
  position: absolute;
  left: 10px;
  color: var(--text-muted);
  font-size: 12px;
}

.search-input {
  padding: 6px 12px 6px 32px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  color: var(--text-primary);
  border-radius: 8px;
  font-size: 12px;
  width: 100%;
  transition: all 0.3s ease;
}

.search-input:focus {
  outline: none;
  border-color: var(--accent-blue);
  box-shadow: 0 0 0 2px rgba(59, 130, 246, 0.2);
}

.goals-list {
  flex: 1;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 8px;
  min-height: 200px;
  max-height: 400px;
}

/* Goal Set Info */
.goal-set-info {
  margin-bottom: 12px;
}

.info-badge {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 6px 10px;
  background-color: rgba(59, 130, 246, 0.1);
  border: 1px solid var(--accent-blue);
  border-radius: 8px;
  font-size: 12px;
  color: var(--accent-blue);
}

.info-badge i {
  font-size: 12px;
}

.clear-btn {
  background: none;
  border: none;
  color: var(--accent-blue);
  cursor: pointer;
  padding: 2px;
  border-radius: 4px;
  transition: all 0.3s ease;
  margin-left: 4px;
}

.clear-btn:hover {
  background-color: rgba(59, 130, 246, 0.2);
}

.goal-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  transition: all 0.3s ease;
  min-height: 60px;
  overflow: hidden;
}

.goal-item:hover {
  background-color: rgba(255, 255, 255, 0.05);
  border-color: var(--accent-blue);
}

.goal-info {
  flex: 1;
  min-width: 0;
  overflow: hidden;
}

.goal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 4px;
  gap: 8px;
}

.goal-name {
  font-weight: 500;
  color: var(--text-primary);
  font-size: 12px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  flex: 1;
  min-width: 0;
}

.goal-badge {
  background-color: var(--accent-blue);
  color: white;
  padding: 2px 6px;
  border-radius: 10px;
  font-size: 9px;
  font-weight: 600;
  flex-shrink: 0;
}

.goal-coordinates {
  display: flex;
  gap: 8px;
  margin-bottom: 4px;
  font-size: 11px;
  color: var(--text-secondary);
  flex-wrap: wrap;
}

.coord-item {
  display: flex;
  align-items: center;
  gap: 4px;
}

.goal-meta {
  display: flex;
  gap: 12px;
  font-size: 11px;
  color: var(--text-muted);
  flex-wrap: wrap;
}

.goal-set {
  display: flex;
  align-items: center;
  gap: 4px;
}

.goal-actions {
  display: flex;
  gap: 4px;
  flex-shrink: 0;
  margin-left: 8px;
}

/* Pagination */
.pagination {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 10px;
  margin-top: 10px;
  padding-top: 10px;
  border-top: 1px solid var(--border-color);
  flex-shrink: 0;
}

.pagination-btn {
  width: 28px;
  height: 28px;
  border: 1px solid var(--border-color);
  background-color: rgba(255, 255, 255, 0.05);
  color: var(--text-secondary);
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: all 0.3s ease;
}

.pagination-btn:hover:not(:disabled) {
  background-color: rgba(255, 255, 255, 0.1);
  color: var(--text-primary);
}

.pagination-btn:disabled {
  opacity: 0.3;
  cursor: not-allowed;
}

.page-info {
  font-size: 11px;
  color: var(--text-muted);
}

/* Mission Builder */
.mission-controls {
  display: flex;
  gap: 6px;
  flex-wrap: wrap;
}

.mission-info-card {
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  padding: 12px;
  margin-bottom: 12px;
}

.info-row {
  display: grid;
  grid-template-columns: repeat(1, 1fr);
  gap: 10px;
  margin-bottom: 10px;
}

.info-group {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.info-group label {
  font-size: 10px;
  color: var(--text-muted);
  font-weight: 500;
}

.mission-input,
.mission-select,
.mission-textarea {
  padding: 8px 10px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  color: var(--text-primary);
  border-radius: 6px;
  font-size: 12px;
  width: 100%;
  transition: all 0.3s ease;
}

.mission-input:focus,
.mission-select:focus,
.mission-textarea:focus {
  outline: none;
  border-color: var(--accent-blue);
  box-shadow: 0 0 0 2px rgba(59, 130, 246, 0.2);
}

.mission-textarea {
  resize: vertical;
  min-height: 60px;
  font-family: inherit;
}

.mission-sequence {
  flex: 1;
  display: flex;
  flex-direction: column;
  min-height: 0;
}

.sequence-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 10px;
  flex-wrap: wrap;
  gap: 8px;
}

.sequence-header h4 {
  margin: 0;
  font-size: 13px;
  color: var(--text-primary);
}

.sequence-actions {
  display: flex;
  gap: 4px;
}

.sequence-list {
  flex: 1;
  overflow-y: auto;
  border: 1px solid var(--border-color);
  border-radius: 8px;
  background-color: rgba(255, 255, 255, 0.03);
  min-height: 150px;
  max-height: 300px;
}

.sequence-item {
  padding: 10px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  transition: all 0.3s ease;
}

.sequence-item:last-child {
  border-bottom: none;
}

.sequence-item:hover {
  background-color: rgba(255, 255, 255, 0.05);
}

.seq-item-header {
  display: flex;
  align-items: center;
  gap: 8px;
  flex-wrap: wrap;
}

.seq-number {
  display: flex;
  align-items: center;
  gap: 6px;
  flex-shrink: 0;
}

.step-badge {
  width: 22px;
  height: 22px;
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  color: white;
  border-radius: 5px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 10px;
  font-weight: 600;
  flex-shrink: 0;
}

.drag-handle {
  color: var(--text-muted);
  cursor: grab;
  padding: 4px;
  display: flex;
  align-items: center;
}

.drag-handle:hover {
  color: var(--text-primary);
}

.seq-info {
  flex: 1;
  min-width: 150px;
  overflow: hidden;
}

.seq-name {
  font-weight: 500;
  color: var(--text-primary);
  font-size: 12px;
  margin-bottom: 2px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.seq-coords {
  font-size: 11px;
  color: var(--text-muted);
  font-family: 'Courier New', monospace;
  overflow: hidden;
  text-overflow: ellipsis;
}

.seq-actions {
  display: flex;
  gap: 2px;
  flex-shrink: 0;
}

/* Goal Trigger Styles */
.goal-trigger-section {
  margin-top: 12px;
  padding: 12px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  border-left: 3px solid var(--accent-blue);
}

.trigger-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 12px;
  flex-wrap: wrap;
}

.trigger-label {
  font-weight: 600;
  font-size: 12px;
  color: var(--text-primary);
}

.trigger-hint {
  font-size: 10px;
  color: var(--text-muted);
}

.trigger-options {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.trigger-option {
  display: flex;
  flex-direction: column;
  gap: 8px;
  padding: 10px;
  background-color: rgba(255, 255, 255, 0.02);
  border-radius: 6px;
  border: 1px solid transparent;
  transition: all 0.3s ease;
}

.trigger-option:hover {
  border-color: var(--border-color);
}

.trigger-label-option {
  display: flex;
  align-items: center;
  gap: 8px;
  cursor: pointer;
  padding: 4px;
  border-radius: 4px;
  transition: all 0.3s ease;
}

.trigger-label-option:hover {
  background-color: rgba(255, 255, 255, 0.05);
}

.trigger-label-option i {
  width: 16px;
  color: var(--text-muted);
  font-size: 14px;
}

.trigger-label-option span:first-of-type {
  font-weight: 500;
  color: var(--text-primary);
  font-size: 12px;
  min-width: 80px;
}

.trigger-desc {
  font-size: 10px;
  color: var(--text-muted);
  flex: 1;
}

.trigger-radio {
  display: none;
}

.trigger-radio:checked+.trigger-label-option {
  background-color: rgba(59, 130, 246, 0.1);
  border: 1px solid var(--accent-blue);
}

.trigger-radio:checked+.trigger-label-option i {
  color: var(--accent-blue);
}

.timer-input,
.sensor-input {
  margin-left: 24px;
  display: flex;
  gap: 8px;
  align-items: center;
  flex-wrap: wrap;
}

.time-input,
.condition-input,
.sensor-select,
.timeout-input,
.retry-input,
.failure-select {
  padding: 6px 10px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  color: var(--text-primary);
  border-radius: 6px;
  font-size: 12px;
  max-width: 200px;
}

.time-unit {
  font-size: 11px;
  color: var(--text-muted);
}

.advanced-trigger {
  margin-top: 12px;
  border-top: 1px solid var(--border-color);
  padding-top: 12px;
}

.advanced-toggle {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px;
  background-color: rgba(255, 255, 255, 0.03);
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.3s ease;
}

.advanced-toggle:hover {
  background-color: rgba(255, 255, 255, 0.05);
}

.advanced-toggle span {
  font-size: 11px;
  color: var(--text-secondary);
  font-weight: 500;
}

.advanced-content {
  margin-top: 8px;
  padding: 12px;
  background-color: rgba(0, 0, 0, 0.2);
  border-radius: 6px;
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.advanced-content .form-group {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.advanced-content label {
  font-size: 10px;
  color: var(--text-muted);
  font-weight: 500;
}

.empty-sequence {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 30px 20px;
  text-align: center;
  color: var(--text-muted);
  flex: 1;
}

.empty-sequence i {
  font-size: 24px;
  margin-bottom: 12px;
  opacity: 0.5;
}

.empty-sequence p {
  margin: 0;
  font-size: 12px;
  max-width: 200px;
}

.mission-stats {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
  margin-top: 12px;
}

.stat-card {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 10px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  min-height: 56px;
  overflow: hidden;
}

.stat-icon {
  width: 32px;
  height: 32px;
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 12px;
  flex-shrink: 0;
}

.stat-icon.success {
  background-color: rgba(16, 185, 129, 0.1);
  color: var(--accent-green);
}

.stat-icon.warning {
  background-color: rgba(245, 158, 11, 0.1);
  color: var(--accent-yellow);
}

.stat-icon.info {
  background-color: rgba(59, 130, 246, 0.1);
  color: var(--accent-blue);
}

.stat-content {
  flex: 1;
  min-width: 0;
}

.stat-value {
  font-weight: 700;
  color: var(--text-primary);
  font-size: 14px;
  line-height: 1;
  overflow: hidden;
  text-overflow: ellipsis;
}

.stat-label {
  font-size: 10px;
  color: var(--text-muted);
  margin-top: 2px;
  overflow: hidden;
  text-overflow: ellipsis;
}

/* Missions Panel */
.missions-list {
  flex: 1;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 6px;
  min-height: 150px;
  max-height: 300px;
}

.mission-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  transition: all 0.3s ease;
  cursor: pointer;
  min-height: 56px;
  overflow: hidden;
}

.mission-item:hover {
  background-color: rgba(255, 255, 255, 0.05);
  border-color: var(--accent-blue);
}

.mission-item.active {
  background-color: rgba(59, 130, 246, 0.1);
  border-color: var(--accent-blue);
}

.mission-info {
  flex: 1;
  min-width: 0;
  overflow: hidden;
}

.mission-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 4px;
  gap: 8px;
}

.mission-name {
  font-weight: 500;
  color: var(--text-primary);
  font-size: 12px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  flex: 1;
  min-width: 0;
}

.mission-status {
  font-size: 9px;
  padding: 2px 6px;
  border-radius: 8px;
  font-weight: 600;
  flex-shrink: 0;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  background-color: rgba(16, 185, 129, 0.1);
  color: #10b981;
}

.mission-details {
  display: flex;
  gap: 10px;
  font-size: 10px;
  color: var(--text-secondary);
  overflow: hidden;
}

.mission-category,
.mission-goals {
  display: flex;
  align-items: center;
  gap: 4px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.mission-date {
  font-size: 10px;
  color: var(--text-muted);
  display: flex;
  align-items: center;
  gap: 4px;
  margin-top: 4px;
}

.mission-actions {
  display: flex;
  gap: 4px;
  flex-shrink: 0;
  margin-left: 8px;
}

.execution-content {
  display: flex;
  flex-direction: column;
  gap: 12px;
  height: 100%;
}

.progress-container {
  padding: 12px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  flex-shrink: 0;
}

.progress-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
  font-size: 12px;
  color: var(--text-primary);
}

.progress-bar {
  height: 6px;
  background-color: rgba(255, 255, 255, 0.1);
  border-radius: 3px;
  overflow: hidden;
  width: 100%;
}

.progress-fill {
  height: 100%;
  background: linear-gradient(90deg, var(--accent-blue), var(--accent-purple));
  border-radius: 3px;
  transition: width 0.3s ease;
  max-width: 100%;
}

.execution-info {
  display: flex;
  flex-direction: column;
  gap: 10px;
  flex-shrink: 0;
}

.execution-stats {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 10px;
}

.stat-item {
  display: flex;
  flex-direction: column;
  gap: 2px;
  padding: 8px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 6px;
  text-align: center;
}

.stat-label {
  font-size: 10px;
  color: var(--text-muted);
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.stat-value {
  font-weight: 600;
  color: var(--text-primary);
  font-size: 14px;
}

.current-goal-info {
  padding: 10px;
  background-color: rgba(255, 255, 255, 0.03);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  flex-shrink: 0;
}

.goal-title {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
  font-size: 12px;
  color: var(--text-primary);
}

.goal-title span:first-child {
  font-weight: 500;
}

.goal-coords {
  display: flex;
  gap: 12px;
  font-size: 11px;
  color: var(--text-secondary);
  font-family: 'Courier New', monospace;
}

.coord {
  flex: 1;
  text-align: center;
  padding: 4px;
  background-color: rgba(255, 255, 255, 0.05);
  border-radius: 4px;
  overflow: hidden;
  text-overflow: ellipsis;
}

.trigger-info {
  margin-top: 8px;
  padding: 6px;
  background-color: rgba(245, 158, 11, 0.1);
  border: 1px solid rgba(245, 158, 11, 0.3);
  border-radius: 4px;
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 11px;
  color: var(--accent-yellow);
}

.small-btn {
  padding: 3px 8px;
  background-color: var(--accent-green);
  color: white;
  border: none;
  border-radius: 3px;
  font-size: 10px;
  cursor: pointer;
  margin-left: auto;
}

.execution-controls {
  display: flex;
  flex-direction: column;
  gap: 8px;
  flex-shrink: 0;
}

.control-row {
  display: grid;
  grid-template-columns: 1fr;
  gap: 6px;
}

.control-btn {
  padding: 10px;
  border: none;
  border-radius: 6px;
  font-weight: 500;
  font-size: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  transition: all 0.3s ease;
  cursor: pointer;
  min-height: 36px;
  width: 100%;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.control-btn.primary {
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  color: white;
  border: none;
}

.control-btn.primary:hover:not(:disabled) {
  background: linear-gradient(135deg, var(--accent-purple), var(--accent-blue));
  box-shadow: 0 6px 20px rgba(59, 130, 246, 0.4);
}

.control-btn.warning {
  background-color: rgba(245, 158, 11, 0.1);
  color: var(--accent-yellow);
  border: 1px solid rgba(245, 158, 11, 0.3);
}

.control-btn.success {
  background-color: rgba(16, 185, 129, 0.1);
  color: var(--accent-green);
  border: 1px solid rgba(16, 185, 129, 0.3);
}

.control-btn.danger {
  background-color: rgba(239, 68, 68, 0.1);
  color: var(--accent-red);
  border: 1px solid rgba(239, 68, 68, 0.3);
}

.control-btn.small {
  min-height: 30px;
  padding: 6px;
  font-size: 11px;
}

.control-btn:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
}

.control-btn:disabled {
  opacity: 0.3;
  cursor: not-allowed;
  transform: none !important;
}

.control-row:last-child {
  grid-template-columns: repeat(3, 1fr);
}

.status-badge {
  padding: 4px 8px;
  border-radius: 12px;
  font-size: 11px;
  font-weight: 600;
  flex-shrink: 0;
}

.status-badge.idle {
  background-color: rgba(108, 117, 125, 0.1);
  color: #6c757d;
}

.status-badge.executing {
  background-color: rgba(245, 158, 11, 0.1);
  color: #f59e0b;
}

.status-badge.paused {
  background-color: rgba(23, 162, 184, 0.1);
  color: #17a2b8;
}

.status-badge.stopped {
  background-color: rgba(239, 68, 68, 0.1);
  color: #ef4444;
}

.status-badge.completed {
  background-color: rgba(16, 185, 129, 0.1);
  color: #10b981;
}

.status-badge.batch-executing {
  background-color: rgba(139, 92, 246, 0.1);
  color: #8b5cf6;
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0% {
    opacity: 1;
  }

  50% {
    opacity: 0.7;
  }

  100% {
    opacity: 1;
  }
}

/* Logs Panel */
.logs-panel {
  overflow: hidden;
}

.logs-container {
  flex: 1;
  overflow-y: auto;
  background-color: rgba(0, 0, 0, 0.2);
  border-radius: 8px;
  padding: 10px;
  border: 1px solid var(--border-color);
  min-height: 150px;
  max-height: 250px;
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.log-entry {
  padding: 6px 8px;
  border-radius: 6px;
  font-size: 11px;
  display: flex;
  align-items: center;
  gap: 8px;
  background-color: rgba(255, 255, 255, 0.03);
  border-left: 3px solid transparent;
  min-height: 24px;
  overflow: hidden;
  word-break: break-word;
  max-width: 100%;
}

.log-entry.info {
  border-left-color: var(--accent-blue);
  background-color: rgba(59, 130, 246, 0.05);
}

.log-entry.success {
  border-left-color: var(--accent-green);
  background-color: rgba(16, 185, 129, 0.05);
}

.log-entry.warning {
  border-left-color: var(--accent-yellow);
  background-color: rgba(245, 158, 11, 0.05);
}

.log-entry.error {
  border-left-color: var(--accent-red);
  background-color: rgba(239, 68, 68, 0.05);
}

.log-icon {
  width: 14px;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-muted);
  flex-shrink: 0;
  font-size: 10px;
}

.log-entry.info .log-icon {
  color: var(--accent-blue);
}

.log-entry.success .log-icon {
  color: var(--accent-green);
}

.log-entry.warning .log-icon {
  color: var(--accent-yellow);
}

.log-entry.error .log-icon {
  color: var(--accent-red);
}

.log-time {
  font-size: 10px;
  color: var(--text-muted);
  min-width: 50px;
  flex-shrink: 0;
}

.log-message {
  color: var(--text-primary);
  flex: 1;
  line-height: 1.3;
  word-wrap: break-word;
  overflow-wrap: break-word;
  max-width: calc(100% - 80px);
}

.empty-logs {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 20px;
  text-align: center;
  color: var(--text-muted);
  flex: 1;
}

.empty-logs i {
  font-size: 24px;
  margin-bottom: 8px;
  opacity: 0.5;
}

.empty-logs p {
  margin: 0;
  font-size: 12px;
}

.more-logs {
  text-align: center;
  padding: 6px;
  font-size: 10px;
  color: var(--text-muted);
  background-color: rgba(255, 255, 255, 0.03);
  border-radius: 4px;
  margin-top: 4px;
  border: 1px dashed var(--border-color);
}

/* Empty States */
.empty-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 20px;
  text-align: center;
  color: var(--text-muted);
  flex: 1;
}

.empty-state i {
  font-size: 24px;
  margin-bottom: 8px;
  opacity: 0.5;
}

.empty-state p {
  margin: 0 0 12px 0;
  font-size: 12px;
  max-width: 180px;
}

/* Manual Confirmation Modal */
.manual-confirmation-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.8);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 2000;
  padding: 16px;
}

.manual-confirmation-modal {
  background-color: var(--card-bg);
  border: 1px solid var(--border-color);
  border-radius: 12px;
  padding: 20px;
  max-width: 400px;
  width: 90%;
  text-align: center;
}

.confirmation-title {
  font-size: 16px;
  color: var(--text-primary);
  margin-bottom: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
}

.confirmation-message {
  font-size: 12px;
  color: var(--text-secondary);
  margin-bottom: 20px;
}

.confirmation-buttons {
  display: flex;
  gap: 12px;
  justify-content: center;
  flex-wrap: wrap;
}

.confirm-btn {
  padding: 10px 20px;
  border: none;
  border-radius: 6px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.3s ease;
  min-width: 140px;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  font-size: 14px;
}

.confirm-btn.success {
  background-color: var(--accent-green);
  color: white;
}

.confirm-btn.danger {
  background-color: var(--accent-red);
  color: white;
}

.confirm-btn:hover {
  opacity: 0.9;
  transform: translateY(-2px);
}

/* Modal Styles */
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.7);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
  padding: 16px;
}

.modal-content {
  background-color: var(--card-bg);
  border: 1px solid var(--border-color);
  border-radius: 12px;
  width: 100%;
  max-width: 400px;
  max-height: 90vh;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px;
  border-bottom: 1px solid var(--border-color);
  flex-shrink: 0;
}

.modal-header h3 {
  margin: 0;
  font-size: 16px;
  color: var(--text-primary);
  display: flex;
  align-items: center;
  gap: 6px;
}

.modal-close {
  background: none;
  border: none;
  color: var(--text-muted);
  font-size: 24px;
  cursor: pointer;
  padding: 0;
  width: 30px;
  height: 30px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 6px;
  transition: all 0.3s ease;
  flex-shrink: 0;
}

.modal-close:hover {
  background-color: rgba(255, 255, 255, 0.1);
  color: var(--text-primary);
}

.modal-body {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.form-group {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.form-group label {
  font-size: 12px;
  color: var(--text-secondary);
  font-weight: 500;
}

.modal-input,
.modal-select {
  padding: 10px;
  background-color: rgba(255, 255, 255, 0.05);
  border: 1px solid var(--border-color);
  color: var(--text-primary);
  border-radius: 6px;
  font-size: 12px;
  width: 100%;
  transition: all 0.3s ease;
}

.modal-input:focus,
.modal-select:focus {
  outline: none;
  border-color: var(--accent-blue);
  box-shadow: 0 0 0 2px rgba(59, 130, 246, 0.2);
}

.color-picker {
  width: 100%;
  height: 40px;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  background: rgba(255, 255, 255, 0.05);
  cursor: pointer;
}

.icon-selection {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 6px;
}

.icon-option {
  width: 36px;
  height: 36px;
  border: 2px solid var(--border-color);
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: all 0.3s ease;
  font-size: 14px;
  color: var(--text-muted);
}

.icon-option:hover {
  border-color: var(--accent-blue);
  color: var(--text-primary);
}

.icon-option.selected {
  border-color: var(--accent-blue);
  background-color: rgba(59, 130, 246, 0.1);
  color: var(--accent-blue);
}

.modal-footer {
  display: flex;
  justify-content: flex-end;
  gap: 8px;
  padding: 16px;
  border-top: 1px solid var(--border-color);
  flex-shrink: 0;
}

.modal-btn {
  padding: 10px 16px;
  border: none;
  border-radius: 6px;
  font-weight: 500;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.3s ease;
  display: flex;
  align-items: center;
  gap: 4px;
  min-width: 100px;
  justify-content: center;
  flex-shrink: 0;
}

.modal-btn.secondary {
  background-color: rgba(255, 255, 255, 0.05);
  color: var(--text-secondary);
  border: 1px solid var(--border-color);
}

.modal-btn.secondary:hover {
  background-color: rgba(255, 255, 255, 0.1);
  color: var(--text-primary);
}

.modal-btn.primary {
  background: linear-gradient(135deg, var(--accent-blue), var(--accent-purple));
  color: white;
}

.modal-btn.primary:hover:not(:disabled) {
  box-shadow: 0 6px 20px rgba(59, 130, 246, 0.4);
  transform: translateY(-2px);
}

.modal-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
  transform: none !important;
}

/* Scrollbar Styling */
::-webkit-scrollbar {
  width: 6px;
  height: 6px;
}

::-webkit-scrollbar-track {
  background: rgba(255, 255, 255, 0.05);
  border-radius: 3px;
}

::-webkit-scrollbar-thumb {
  background: rgba(255, 255, 255, 0.2);
  border-radius: 3px;
}

::-webkit-scrollbar-thumb:hover {
  background: rgba(255, 255, 255, 0.3);
}

/* Animation for spinner */
@keyframes spin {
  from {
    transform: rotate(0deg);
  }

  to {
    transform: rotate(360deg);
  }
}

.fa-spin {
  animation: spin 1s linear infinite;
}

/* Responsive Design */
@media (min-width: 768px) {
  .mission-control {
    padding: 16px;
  }

  .header-section {
    flex-direction: row;
    padding: 20px;
  }

  .header-actions {
    flex-wrap: nowrap;
  }

  .header-btn {
    flex: none;
  }

  .main-grid {
    grid-template-columns: repeat(2, 1fr);
    grid-template-rows: auto auto;
    height: calc(100vh - 140px);
  }

  .left-column {
    grid-column: 1;
    grid-row: 1;
  }

  .middle-column {
    grid-column: 2;
    grid-row: 1 / span 2;
  }

  .right-column {
    grid-column: 1;
    grid-row: 2;
  }

  .info-row {
    grid-template-columns: repeat(2, 1fr);
  }

  .goal-item {
    flex-wrap: nowrap;
  }

  .mission-item {
    flex-wrap: nowrap;
  }

  .logs-container {
    max-height: 200px;
  }

  .control-row {
    grid-template-columns: 1fr;
  }

  .control-row:last-child {
    grid-template-columns: repeat(3, 1fr);
  }
}

@media (min-width: 1024px) {
  .main-grid {
    grid-template-columns: 300px 1fr 300px;
    grid-template-rows: 1fr;
  }

  .left-column {
    grid-column: 1;
    grid-row: 1;
  }

  .middle-column {
    grid-column: 2;
    grid-row: 1;
  }

  .right-column {
    grid-column: 3;
    grid-row: 1;
  }

  .header-text h1 {
    font-size: 22px;
  }

  .subtitle {
    font-size: 13px;
  }

  .logs-container {
    max-height: 180px;
  }

  .control-row {
    grid-template-columns: 1fr;
  }
}

@media (min-width: 1200px) {
  .mission-control {
    padding: 20px;
  }

  .main-grid {
    grid-template-columns: 320px 1fr 320px;
  }

  .card {
    padding: 20px;
  }
}

/* Touch-friendly improvements */
@media (hover: none) and (pointer: coarse) {

  .action-btn,
  .header-btn,
  .control-btn,
  .modal-btn {
    min-height: 44px;
    padding: 12px 16px;
  }

  .icon-btn {
    width: 44px;
    height: 44px;
  }

  .sequence-item {
    padding: 12px;
  }

  .goal-item,
  .mission-item,
  .category-item {
    padding: 12px;
  }

  .search-input,
  .mission-input,
  .mission-select,
  .mission-textarea {
    font-size: 16px;
    min-height: 44px;
  }
}

/* Extra small devices */
@media (max-width: 480px) {
  .mission-control {
    padding: 8px;
    gap: 12px;
  }

  .header-section {
    padding: 12px;
  }

  .header-icon {
    width: 40px;
    height: 40px;
    font-size: 18px;
  }

  .header-text h1 {
    font-size: 18px;
  }

  .subtitle {
    font-size: 11px;
  }

  .header-btn {
    font-size: 11px;
    padding: 6px 10px;
  }

  .card {
    padding: 12px;
  }

  .panel-header h3 {
    font-size: 13px;
  }

  .execution-stats {
    grid-template-columns: 1fr;
  }

  .goal-coords {
    flex-direction: column;
    gap: 6px;
  }

  .coord {
    width: 100%;
  }

  .control-row {
    grid-template-columns: 1fr;
  }

  .control-row:last-child {
    grid-template-columns: repeat(3, minmax(0, 1fr));
  }

  .control-btn {
    font-size: 11px;
    padding: 8px 4px;
  }

  .log-message {
    font-size: 10px;
  }

  .mission-item {
    flex-wrap: wrap;
  }

  .mission-actions {
    width: 100%;
    justify-content: flex-end;
    margin-top: 8px;
    margin-left: 0;
  }
}

/* Tablet specific adjustments */
@media (min-width: 768px) and (max-width: 1023px) {
  .execution-content {
    gap: 10px;
  }

  .execution-stats {
    grid-template-columns: repeat(2, 1fr);
  }

  .control-btn {
    font-size: 11px;
  }

  .log-message {
    font-size: 11px;
  }

  .middle-column {
    max-height: calc(100vh - 160px);
  }
}
</style>