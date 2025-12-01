<script setup lang="ts">
import { ref } from 'vue';
import { useRouter } from 'vue-router';
import { createUserWithEmailAndPassword } from 'firebase/auth';
import { auth } from '../firebase';

const router = useRouter();

const name = ref('');
const email = ref('');
const password = ref('');
const errorMessage = ref('');
const showError = ref(false);

const handleRegister = async () => {
    if (!name.value || !email.value || !password.value) {
        errorMessage.value = 'Semua field harus diisi!';
        showError.value = true;
        return;
    }

    try {
        const userCredential = await createUserWithEmailAndPassword(auth, email.value, password.value);
        const user = userCredential.user;

        // (Opsional) Simpan username ke localStorage
        localStorage.setItem('user', JSON.stringify({
            uid: user.uid,
            email: user.email,
            name: name.value
        }));

        alert(`Registrasi berhasil! Selamat datang, ${name.value}`);
        router.push('/');
    } catch (error: any) {
        errorMessage.value = error.message;
        showError.value = true;
    }
};

const goToLogin = () => {
    router.push('/');
};
</script>

<template>
    <div class="container min-w-full min-h-screen flex justify-center items-center">
        <div
            class="form-container w-full max-w-3xl h-auto grid grid-cols-1 md:grid-cols-2 rounded-xl bg-white shadow-2xl overflow-hidden">
            <div class="flex flex-col">
                <header
                    class="w-full flex flex-col md:flex-row justify-between h-auto md:h-25 bg-gradient-to-r from-cyan-500 to-blue-500 rounded-tl-xl items-center font-bold p-3 text-white mb-3">
                    <div class="logo bg-white rounded-full w-15 h-15 p-2 flex justify-center items-center ml-0 md:ml-3">
                        <img src="../assets/img/logo-polman.png" class="w-10 h-11" alt="">
                    </div>
                    <div class="title w-full md:w-1/2 mt-2 md:mt-0">
                        <p class="text-center md:text-right text-lg">WELCOME TO AMR DASHBOARD</p>
                    </div>
                </header>
                <form @submit.prevent="handleRegister" class="flex flex-col gap-2 px-6 md:px-8 py-6 justify-center items-center">
                    <label class="w-full text-left font-bold" for="username">Username</label>
                    <input class="w-full h-12 border border-black p-3 rounded-2xl focus:outline-none focus:ring-2 focus:ring-cyan-500" type="text" v-model="name" id="username" name="username"
                        placeholder="input username">
                    <label class="w-full text-left font-bold" for="email">Email</label>
                    <input class="w-full h-12 border border-black p-3 rounded-2xl focus:outline-none focus:ring-2 focus:ring-cyan-500" type="email" v-model="email" id="email" name="email"
                        placeholder="input email">
                    <label class="w-full text-left font-bold" for="password">Password</label>
                    <input class="w-full h-12 border border-black p-2 rounded-2xl focus:outline-none focus:ring-2 focus:ring-cyan-500" type="password" v-model="password" id="password"
                        name="password">
                    <a @click.prevent="goToLogin" href="#" class="w-full text-right mb-3 mt-1 text-cyan-600 underline">Login akun</a>
                    <button class="submit-btn p-3 rounded-xl w-full md:w-1/2 text-white font-bold cursor-pointer"
                        type="submit">Register</button>
                </form>
                <!-- Notifikasi Error -->
                <div v-if="showError" class="bg-red-500 text-white p-3 rounded-lg mt-3">
                    {{ errorMessage }}
                </div>
            </div>
            <div class="sideimg hidden md:flex flex-col gap-3 justify-center items-center">
                <div class="logo bg-white rounded-full w-20 h-20 p-2 flex justify-center items-center">
                    <img src="../assets/img/logo-polman.png" class="w-12 h-12" alt="">
                </div>
                <p class="w-full font-bold text-white text-center">AUTONOMOUS MOBILE ROBOT</p>
            </div>
        </div>
    </div>
</template>

<style scoped>
.container {
    background: linear-gradient(to bottom right, #03AEC0, #00525B);
    padding: 0;
}

.submit-btn {
    background: #03A1B3;
}

.sideimg {
    background: url('../assets/img/sideimg.png') no-repeat center;
    position: relative;
}

.sideimg::before {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    background: black;
    opacity: 0.3;
    z-index: 1;
}

.sideimg > * {
    position: relative;
    z-index: 2;
}

@media (max-width: 768px) {
    .container {
        padding: 1rem;
    }
}
</style>