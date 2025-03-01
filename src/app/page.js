import Background from "@/components/chessboard/Background";
import Chess from "@/components/chessboard/CustomChessBoard";
import React from "react";

const page = () => {
  return (
    <div className="relative max-w-[450px] w-full h-screen bg-blackColor flex items-center justify-center overflow-hidden p-3">
      <Background />

      <div className="w-full relative z-10 text-white text-3xl font-bold">
        <Chess />
      </div>
    </div>
  );
};

export default page;
