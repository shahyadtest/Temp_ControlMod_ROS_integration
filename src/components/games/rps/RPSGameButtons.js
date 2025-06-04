"use client";

import { toFarsiNumber } from "@/helper/helper";
import { postData, siteURL } from "@/services/API";
import { Button } from "@heroui/react";
import Image from "next/image";
import { useRouter } from "next/navigation";
import React, { useEffect, useState } from "react";
import toast, { Toaster } from "react-hot-toast";
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
  const [resultMessage, setResultMessage] = useState(null);
  const [saveResultLoading, setSaveResultLoading] = useState(false);
  const [playersMoves, setPlayersMoves] = useState([]);
  const [playersMovesImage, setPlayersMovesImage] = useState({
    me: "/rps/hand.svg",
    opponent: "/rps/hand.svg",
    key: 0,
  });

  const [points, setPoints] = useState({
    me: 0,
    opponent: 0,
  });

  const router = useRouter();

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
      setSelectedMove({});
      setPlayersMoves((prev) => [...prev, gameMoves]);

      // set points & result message
      if (winner === user._id) {
        setPoints((prev) => ({ ...prev, me: prev.me + 10 }));

        setResultMessage("win");
      } else if (winner === roomInfo.opponent._id) {
        setPoints((prev) => ({ ...prev, opponent: prev.opponent + 10 }));

        setResultMessage("lose");
      } else {
        setResultMessage("draw");
      }

      setTimeout(() => {
        setResultMessage(null);
      }, 4000);

      // set players moves
      setPlayersMovesImage((prev) => ({
        me: movesHand[gameMoves[user._id]],
        opponent: movesHand[gameMoves[roomInfo.opponent._id]],
        key: prev.key + 1,
      }));
    });

    // finish the game
    if (points.me === 100 || points.opponent === 100) {
      if (points.me === 100) {
        // save game result
        setSaveResultLoading(true);

        postData("/rps/save-result", {
          roomId,
          winner: user._id,
          moves: playersMoves,
        })
          .then(() => {
            setSaveResultLoading(false);
          })
          .catch((err) => {
            setSaveResultLoading(false);
            toast.error(err?.response?.data?.error || "خطا هنگام ذخیره بازی", {
              duration: 4000,
              style: {
                borderRadius: "10px",
                background: "#040e1c",
                color: "#fff",
                fontSize: "14px",
              },
            });
          });

        setGameResult("you win");
      } else {
        setGameResult("you lose");
      }
    }

    return () => {
      socket.off();
    };
  }, [points]);

  const userMoveHandler = (move) => {
    setSelectedMove(move);
    socket.emit("makeMove", { roomId, move: move.name, userId: user._id });
  };

  const backHomePageHandler = () => {
    // toast.success("درحال انتقال به صفحه اصلی...", {
    //   duration: 4000,
    //   style: {
    //     borderRadius: "10px",
    //     background: "#040e1c",
    //     color: "#fff",
    //     fontSize: "14px",
    //   },
    // });

    router.push("/");
  };

  console.log(playersMoves)

  return (
    <div className="w-full h-full relative max-w-[450px]">
      <Toaster />

      {/* round result message */}
      {!gameResult ? (
        <div
          className={`fixed ${
            resultMessage && !gameResult ? "opacity-100 visible" : "opacity-0 invisible"
          } w-full max-w-[450px] h-full flex justify-center items-center text-2xl font-black z-[60] top-0 right-0 bottom-0 bg-black bg-opacity-75 transition-all duration-300`}
        >
          {resultMessage === "win" ? (
            <span className="text-success">آفرین! این دست و بردی</span>
          ) : resultMessage === "lose" ? (
            <span className="text-red-600">ای وای! این دست و باختی</span>
          ) : resultMessage === "draw" ? (
            <span className="text-gray-200">این دست مساوی شد!</span>
          ) : (
            ""
          )}
        </div>
      ) : (
        ""
      )}

      {/* game result message */}
      <div
        className={`fixed ${
          gameResult ? "opacity-100 visible" : "opacity-0 invisible"
        } w-full max-w-[450px] h-full flex flex-col gap-4 justify-center items-center z-[60] top-0 right-0 bottom-0 bg-black bg-opacity-85 transition-all duration-300`}
      >
        {gameResult === "you win" ? (
          <span className="text-success text-2xl font-black">
            آفرین! این بازی و بردی
          </span>
        ) : gameResult === "you lose" ? (
          <span className="text-red-600 text-2xl font-black">
            ای وای! این بازی و باختی
          </span>
        ) : (
          ""
        )}

        <div className="w-full flex justify-center items-center gap-16 text-sm text-gray-200">
          <span>امتیاز شما: {toFarsiNumber(points.me / 10)}</span>
          <span>امتیاز حریف: {toFarsiNumber(points.opponent / 10)}</span>
        </div>

        <p className="text-xs font-normal text-gray-300 -mt-1">
          برای بازگشت به صفحه اصلی و شروع بازی جدید کلیک کن
        </p>

        <Button
          isLoading={saveResultLoading}
          onClick={backHomePageHandler}
          className="bg-blueColor"
        >
          {saveResultLoading ? "لطفا صبر کنید..." : "صفحه اصلی"}
        </Button>
      </div>

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
          src={playersMovesImage.me}
          width={350}
          height={800}
          className="w-full max-w-20 absolute -bottom-20 right-0 slide-up"
          alt="hand"
          key={playersMovesImage.key}
        />
      </div>

      {/* opponent move hand */}
      <div className="">
        <Image
          src={playersMovesImage.opponent}
          width={350}
          height={800}
          className="w-full max-w-20 absolute -top-20 left-0 -scale-x-100 -scale-y-100 slide-down"
          alt="hand"
          key={playersMovesImage.key}
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
            className="w-full bg-redColor transition-all duration-300 rounded-b-full"
          ></div>

          {/* opponent */}
          <div className="w-full h-full flex items-end">
            <div
              style={{ height: `${points.me}%` }}
              className="w-full bg-blueColor transition-all duration-300 rounded-t-full"
            ></div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RPSGameButtons;
