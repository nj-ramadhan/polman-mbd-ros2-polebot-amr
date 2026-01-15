import { createRouter, createWebHashHistory } from 'vue-router'
import AppLayout from '../components/AppLayout.vue'

const routes = [
  {
    path: '/',
    component: AppLayout,
    children: [
      {
        path: '',
        redirect: '/map'
      },
      {
        path: 'map',
        name: 'GoalPlanning',
        component: () => import('../components/GoalPlanning.vue')
      },
      {
        path: 'mission',
        name: 'MissionControl',
        component: () => import('../components/MissionControl.vue')
      }
    ]
  }
]

const router = createRouter({
  history: createWebHashHistory(),
  routes
})

export default router