import { create } from "zustand";

export const useUser = create((set, get) => ({
  user: {},
  setUser: (data) => set({ user: data }),
}));
