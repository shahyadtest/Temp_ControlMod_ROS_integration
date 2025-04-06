import Background from "@/components/chessboard/Background";
import RPSGameButtons from "@/components/games/rps/RPSGameButtons";
import { getUser } from "@/lib/fetchUser";
import { baseURL } from "@/services/API";
import React from "react";

const page = async ({ params }) => {
  const { roomId } = await params;
  const { user } = await getUser();

  const fetchRoom = async () => {
    try {
      const roomInfoRes = await fetch(`${baseURL}/rps/get-room`, {
        method: "POST",
        body: JSON.stringify({ roomId, userId: user._id }),
      });

      return { roomInfo: await roomInfoRes.json() };
    } catch (error) {
      return { roomInfo: error };
    }
  };

  const { roomInfo } = await fetchRoom();

  return (
    <div className="relative max-w-[450px] flex flex-col gap-5 w-full h-screen bg-primaryDarkTheme overflow-hidden p-5">
      <Background />

      <div className="w-full flex justify-between items-center bg-primaryDarkTheme">
        <h1 className="bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent">
          بازی سنگ کاغذ قیچی
        </h1>

        <div className="flex items-center text-sm gap-2">
          <span className="bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent">
            شما
          </span>
          <span>🆚</span>
          <span className="bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent">
            {roomInfo?.opponent?.nickName}
          </span>
        </div>
      </div>

      <RPSGameButtons roomId={roomId} roomInfo={roomInfo} user={user} />
    </div>
  );
};

export default page;
