import Image from "next/image";
import React from "react";

const Chess = () => {
  return (
    <div className="w-full h-36 rounded-2xl p-0.5 group bg-gradient-to-b from-gray-600 hover:from-blueColor">
      <div className="w-full h-full flex flex-col group-hover:text-blueColor justify-center items-center gap-3 bg-secondaryDarkTheme rounded-2xl transition-all duration-300">
        <Image
          src={"/chess.png"}
          width={100}
          height={100}
          alt={"chess"}
          className="size-16"
        />
        <span className="text-xs font-semibold">شطرنج</span>
      </div>
    </div>
  );
};

export default Chess;
