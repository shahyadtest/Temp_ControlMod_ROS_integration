"use client";

import { siteURL } from "@/services/API";
import Image from "next/image";
import React, { useEffect, useState } from "react";
import { io } from "socket.io-client";

const socket = io(siteURL);

const RPSGameButtons = ({ roomId, roomInfo, user }) => {
  const [gameResult, setGameResult] = useState(null);
  const [selectedMove, setSelectedMove] = useState({});
  const [turn, setTurn] = useState(roomInfo?.host?._id === user._id);

  const moves = [
    {
      id: 1,
      name: "rock",
      icon: "/rps/r-icon.svg",
      position: "bottom-2 -ml-[100px]",
    },
    {
      id: 2,
      name: "paper",
      icon: "/rps/p-icon.svg",
      position: "bottom-[60px]",
    },
    {
      id: 3,
      name: "scissors",
      icon: "/rps/s-icon.svg",
      position: "bottom-2 ml-[100px]",
    },
  ];

  useEffect(() => {
    socket.on("turnUpdate", ({ turn }) => {
      console.log(turn);
    });

    socket.on("gameOver", ({ result, winner }) => {
      setGameResult({ result, winner, gameMoves });
    });

    return () => {
      socket.off();
    };
  }, []);

  const userMoveHandler = (move) => {
    setSelectedMove(move);
    socket.emit("makeMove", { roomId, move: move.name, userId: user._id });
  };

  console.log(gameResult);

  return (
    <div className="w-full h-full relative max-w-[450px]">
      <span
        className={`p-2 text-xs ${
          turn ? "bg-success text-success" : "bg-red-600 text-red-600"
        } bg-opacity-15 transition-all duration-300 rounded-xl absolute left-2/4 -translate-x-2/4 bottom-36`}
      >
        {turn ? "نوبت شما" : "نوبت حریف"}
      </span>

      <button
        disabled={!turn}
        className="w-32 h-16 flex justify-center items-center rounded-t-full bg-secondaryDarkTheme absolute left-2/4 -translate-x-2/4 -bottom-5 disabled:opacity-80"
      >
        <Image
          src={"/rps/random-icon.svg"}
          width={100}
          height={100}
          className="size-7 block -mb-3"
        />
      </button>

      {moves.map((item) => (
        <button
          onClick={() => userMoveHandler(item)}
          key={item.id}
          disabled={!turn}
          className={`w-16 h-16 flex justify-center items-center rounded-full absolute left-2/4 -translate-x-2/4 ${
            item.position
          } ${
            item.id === selectedMove?.id
              ? "bg-blueColor"
              : "bg-secondaryDarkTheme"
          } hover:bg-blueColor disabled:opacity-80 transition-all duration-300`}
        >
          <Image
            src={item.icon}
            width={100}
            height={100}
            className="size-8 block"
          />
        </button>
      ))}

      {/* <button className="w-16 h-16 flex justify-center items-center rounded-full bg-secondaryDarkTheme absolute left-2/4 -translate-x-2/4 bottom-2 -ml-[100px]">
        <Image
          src={"/rps/r-icon.svg"}
          width={100}
          height={100}
          className="size-8 block"
        />
      </button>

      <button className="w-16 h-16 flex justify-center items-center rounded-full bg-secondaryDarkTheme absolute left-2/4 -translate-x-2/4 bottom-[60px]">
        <Image
          src={"/rps/p-icon.svg"}
          width={100}
          height={100}
          className="size-8 block"
        />
      </button>

      <button className="w-16 h-16 flex justify-center items-center rounded-full bg-secondaryDarkTheme absolute left-2/4 -translate-x-2/4 bottom-2 ml-[100px]">
        <Image
          src={"/rps/s-icon.svg"}
          width={100}
          height={100}
          className="size-8 block"
        />
      </button> */}
    </div>
  );
};

export default RPSGameButtons;
