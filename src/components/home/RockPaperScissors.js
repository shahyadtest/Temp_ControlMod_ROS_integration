"use client";

import { baseURL, siteURL } from "@/services/API";
import { useUser } from "@/store/useUser";
import { Spinner } from "@heroui/react";
import Image from "next/image";
import { useRouter } from "next/navigation";
import React, { useEffect, useState } from "react";
import toast, { Toaster } from "react-hot-toast";
import io from "socket.io-client";

const socket = io(siteURL);

const RockPaperScissors = ({ user }) => {
  const [loading, setLoading] = useState(false);
  const [gameInfo, setGameInfo] = useState({});
  const router = useRouter();

  useEffect(() => {
    socket.emit("userInfo", {
      userId: user._id,
      userName: user.userName,
      nickName: user.nickName,
    });

    socket.on("onlineUsers", (users) => console.log(users));

    socket.on("gameFound", ({ roomId, opponent, playerTurn }) => {
      setLoading(false);

      toast.success("حریف شما پیدا شد درحال انتقال به بازی...", {
        duration: 4500,
        style: {
          borderRadius: "10px",
          background: "#040e1c",
          color: "#fff",
          fontSize: "14px",
        },
      });

      router.push(`/rps/${roomId}`);

      setGameInfo({ roomId, opponent, playerTurn });
    });

    return () => socket.off();
  }, []);

  const findGameHandler = () => {
    setLoading(true);
    socket.emit("findGame", { userId: user._id });
  };

  const cancelGameFindingHandler = () => {
    setLoading(false);

    socket.emit("cancelGame");
  };

  console.log(gameInfo)

  return (
    <>
      <Toaster />
      {(loading || gameInfo?.roomId) && (
        <div className="w-full h-full fixed max-w-[450px] top-0 flex flex-col justify-center items-center gap-6 bottom-0 right-0 z-50 bg-black bg-opacity-80">
          {loading && (
            <>
              <Spinner
                label="درحال پیدا کردن حریف، منتظر بمانید..."
                classNames={{
                  label: "text-sm",
                  circle1: "border-b-blueColor",
                  circle2: "border-b-blueColor",
                }}
              />

              <button
                onClick={cancelGameFindingHandler}
                className="bg-red-600 bg-opacity-15 text-red-600 text-sm px-3 py-2 rounded-xl"
              >
                لغو بازی
              </button>
            </>
          )}

          {gameInfo.roomId && (
            <div className="flex flex-col items-center gap-16">
              <div className="flex flex-col text-sm items-center gap-2 slide-up-2">
                <Image
                  src={"/avatar.png"}
                  width={100}
                  height={100}
                  alt={"player-1-avatar"}
                  className="size-16 rounded-2xl"
                />

                <span>{gameInfo?.playerTurn?.nickName}</span>
              </div>

              <span className="text-2xl text-success font-black">حریف شما پیدا شد !</span>

              <div className="flex flex-col text-sm items-center gap-2 slide-down-2">
                <Image
                  src={"/avatar.png"}
                  width={100}
                  height={100}
                  alt={"player-1-avatar"}
                  className="size-16 rounded-2xl"
                />

                <span>{gameInfo?.opponent?.nickName}</span>
              </div>
            </div>
          )}
        </div>
      )}

      <button
        onClick={findGameHandler}
        className="w-full h-36 rounded-2xl p-0.5 group bg-gradient-to-b from-gray-600 hover:from-blueColor"
      >
        <div className="w-full h-full flex flex-col group-hover:text-blueColor justify-center items-center gap-3 bg-secondaryDarkTheme rounded-2xl transition-all duration-300">
          <Image
            src={"/rock-paper-scissors.png"}
            width={100}
            height={100}
            alt={"rock-paper-scissors"}
            className="size-16"
          />
          <span className="text-xs font-semibold">سنگ کاغذ قیچی</span>
        </div>
      </button>
    </>
  );
};

export default RockPaperScissors;
