import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  base: "/next/",
  build: {
    emptyOutDir: true,
    outDir: "../web/next",
  },
  plugins: [react()],
});
