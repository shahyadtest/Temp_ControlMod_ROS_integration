import Background from "@/components/shared/Background";
import React from "react";

const page = () => {
  return (
    <div className="relative w-full h-screen bg-blackColor flex items-center justify-center overflow-hidden">
      <Background />

      <div className="relative z-10 text-white text-3xl font-bold">
        Luscent Chess
      </div>
    </div>
  );
};

export default page;
