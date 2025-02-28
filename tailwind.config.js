import { heroui } from "@heroui/react";

/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,ts,jsx,tsx,mdx}",
    "./node_modules/@heroui/theme/dist/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        blueColor: "#3D4AEB",
        redColor: "#B15653",
        blackColor: "#0C0813",
        grayColor: "#373855",
      },
    },
  },

  darkMode: "class",
  plugins: [heroui()],
};
