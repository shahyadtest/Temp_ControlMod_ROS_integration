"use client";

import { toFarsiNumber } from "@/helper/helper";
import { siteURL } from "@/services/API";
import Image from "next/image";
import React, { useEffect, useState } from "react";
import { io } from "socket.io-client";

const socket = io(siteURL);

const movesHand = {
  rock: "/rps/rock.svg",
  paper: "/rps/paper.svg",
  scissors: "/rps/scissors.svg",
};

const RPSGameButtons = ({ roomId, roomInfo, user }) => {
  const [gameResult, setGameResult] = useState(null);
  const [selectedMove, setSelectedMove] = useState({});
  const [turn, setTurn] = useState(roomInfo?.host?._id === user._id);
  const [playersMoves, setPlayersMoves] = useState({
    me: "/rps/hand.svg",
    opponent: "/rps/hand.svg",
  });
  const [points, setPoints] = useState({
    me: 0,
    opponent: 0,
  });

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
    socket.emit("joinRoom", roomId);

    socket.emit("userInfo", {
      userId: user._id,
      userName: user.userName,
      nickName: user.nickName,
    });

    socket.on("waitingForOpponent", ({ currentPlayer }) => {
      if (currentPlayer === user._id) {
        setTurn(false);
      } else {
        setTurn(true);
      }
    });

    socket.on("gameOver", ({ result, winner, gameMoves }) => {
      setGameResult({ result, winner, gameMoves });

      setSelectedMove({});

      // set points
      if (winner === user._id) {
        setPoints((prev) => ({ ...prev, me: prev.me + 10 }));
      } else if (winner === roomInfo.opponent._id) {
        setPoints((prev) => ({ ...prev, opponent: prev.opponent + 10 }));
      }
      console.log(gameMoves[user._id]);
      // set players moves
      setPlayersMoves({
        me: movesHand[gameMoves[user._id]],
        opponent: movesHand[gameMoves[roomInfo.opponent._id]],
      });
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
  console.log(playersMoves);

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
        className="w-32 h-16 flex justify-center items-center bg-secondaryDarkTheme absolute left-2/4 -translate-x-2/4 -bottom-5 disabled:opacity-80 z-50 rounded-t-full"
      >
        <Image
          src={"/rps/random-icon.svg"}
          width={100}
          height={100}
          className="size-7 block -mb-3"
          alt="move"
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
          } hover:bg-blueColor disabled:opacity-80 transition-all duration-300 z-50`}
        >
          <Image
            src={item.icon}
            width={100}
            height={100}
            className="size-8 block"
            alt="move"
          />
        </button>
      ))}

      {/* user move hand */}
      <div className="">
        <Image
          src={playersMoves.me}
          width={350}
          height={800}
          className="w-full max-w-20 absolute -bottom-20 right-0 slide-up"
          alt="hand"
        />
      </div>

      {/* opponent move hand */}
      <div className="">
        <Image
          src={playersMoves.opponent}
          width={350}
          height={800}
          className="w-full max-w-20 absolute -top-20 left-0 -scale-x-100 -scale-y-100 slide-down"
          alt="hand"
        />
      </div>

      {/* ponts */}
      <div className="w-3 h-44 bg-secondaryDarkTheme absolute right-4 top-2/4 -translate-y-3/4">
        <div className="relative w-full h-full grid grid-rows-2">
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="currentColor"
            className="size-4 absolute fill-white top-2/4 -translate-y-2/4 -left-[14px] -scale-x-100"
          >
            <path
              fillRule="evenodd"
              d="M4.5 5.653c0-1.427 1.529-2.33 2.779-1.643l11.54 6.347c1.295.712 1.295 2.573 0 3.286L7.28 19.99c-1.25.687-2.779-.217-2.779-1.643V5.653Z"
              clipRule="evenodd"
            />
          </svg>

          <span className="text-xs absolute top-2/4 -translate-y-2/4 -left-[86px] block min-w-16 pb-0.5">
            برنده بازی 🎉
          </span>

          <div className="absolute -top-[50px] left-2/4 -translate-x-2/4 flex flex-col gap-1 justify-center items-center">
            <span className="text-xs min-w-10 block text-center">
              {toFarsiNumber(points.opponent / 10)} امتیاز
            </span>

            <Image
              src={"/avatar.png"}
              width={100}
              height={100}
              className="min-w-8 size-8 rounded-full"
              alt="user profile"
            />
          </div>

          <div className="absolute -bottom-[50px] left-2/4 -translate-x-2/4 flex flex-col gap-1 justify-center items-center">
            <Image
              src={"/avatar.png"}
              width={100}
              height={100}
              className="min-w-8 size-8 rounded-full"
              alt="user profile"
            />

            <span className="text-xs min-w-10 block text-center">
              {toFarsiNumber(points.me / 10)} امتیاز
            </span>
          </div>

          {/* point progress */}
          {/* me */}
          <div
            style={{ height: `${points.opponent}%` }}
            className="w-full bg-redColor transition-all duration-300"
          ></div>

          {/* opponent */}
          <div className="w-full h-full flex items-end">
            <div
              style={{ height: `${points.me}%` }}
              className="w-full bg-blueColor transition-all duration-300"
            ></div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RPSGameButtons;
