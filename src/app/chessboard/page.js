import Background from "@/components/chessboard/Background";
import CustomChessBoard from "@/components/chessboard/CustomChessBoard";
import React from "react";

const page = () => {
  return (
    <div className="relative max-w-[450px] flex flex-col gap-5 w-full h-screen bg-blackColor items-center justify-center overflow-hidden p-3">
      <Background />
      <Auth />

      <div className="w-full relative z-10 text-white text-3xl font-bold">
        
      </div>
    </div>
  );
};

export default page;
