import { createApp } from 'vue'
import './style.css'
import App from './App.vue'
import 'material-symbols/index.css'
import router from './route'

const app = createApp(App)
app.use(router)
app.mount('#app')
