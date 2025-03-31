import Background from "@/components/chessboard/Background";
import Header from "@/components/header/Header";
import Chess from "@/components/home/Chess";
import Poker from "@/components/home/Poker";
import RockPaperScissors from "@/components/home/RockPaperScissors";
import Navbar from "@/components/navbar/Navbar";
import { getUser } from "@/lib/fetchUser";
import Image from "next/image";
import React from "react";

const page = async () => {
  const { user } = await getUser();
  const boxes = [
    { id: 1, name: "سنگ کاغذ قیچی", icon: "/rock-paper-scissors.png" },
    { id: 2, name: "شطرنج", icon: "/chess.png" },
    { id: 3, name: "پوکر", icon: "/poker.png" },
  ];

  return (
    <div className="relative max-w-[450px] flex flex-col gap-5 w-full h-screen bg-primaryDarkTheme overflow-hidden p-5">
      <Header />
      <Navbar />
      <Background />

      {/* boxes */}
      <div className="w-full grid grid-cols-3 gap-3">
        <RockPaperScissors user={user} />
        <Chess />
        <Poker />
        {/* {boxes.map((item, index) => (
          <div
            key={index}
            className="w-full h-36 rounded-2xl p-0.5 group bg-gradient-to-b from-gray-600 hover:from-blueColor"
          >
            <div className="w-full h-full flex flex-col group-hover:text-blueColor justify-center items-center gap-3 bg-secondaryDarkTheme rounded-2xl transition-all duration-300">
              <Image
                src={item.icon}
                width={100}
                height={100}
                alt={item.name}
                className="size-16"
              />
              <span className="text-xs font-semibold">{item.name}</span>
            </div>
          </div>
        ))} */}
      </div>
    </div>
  );
};

export default page;
