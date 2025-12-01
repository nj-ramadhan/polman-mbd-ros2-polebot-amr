import { initializeApp } from "firebase/app";
import { getAuth } from "firebase/auth";
import { getFirestore } from "firebase/firestore";

const firebaseConfig = {
  apiKey: "AIzaSyCSRHmhjnosCJwzHl-AxeIuhW7zxLG3LOg",
  authDomain: "automationmobilerobot.firebaseapp.com",
  projectId: "automationmobilerobot",
  storageBucket: "automationmobilerobot.firebasestorage.app",
  messagingSenderId: "565216397923",
  appId: "1:565216397923:web:5d51a56149455b77da4656",
  measurementId: "G-5V52PGCK6F"
};

const app = initializeApp(firebaseConfig);

export const auth = getAuth(app);
export const db = getFirestore(app);
