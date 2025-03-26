import Background from "@/components/chessboard/Background";
import Navbar from "@/components/navbar/Navbar";
import React from "react";
import SearchFriends from "./SearchFriends";
import FriendItem from "./FriendItem";
import { Spinner } from "@heroui/react";
import FriendRequests from "./FriendRequests";
import { baseURL } from "@/services/API";
import { cookies } from "next/headers";
import { toFarsiNumber } from "@/helper/helper";

const page = async () => {
  const token = (await cookies()).get("token")?.value;

  const res = await fetch(`${baseURL}/friends/get-all`, {
    method: "GET",
    credentials: "include",
    headers: {
      Authorization: `Bearer ${token}`,
    },
  });

  const data = await res.json();

  return (
    <div className="relative max-w-[450px] flex flex-col items-center gap-5 w-full h-screen bg-primaryDarkTheme overflow-hidden p-5">
      <Navbar />
      <Background />

      <div className="w-full flex flex-col items-center gap-1">
        <h1 className="text-2xl font-black bg-gradient-to-b from-white to-gray-600 bg-clip-text text-transparent">
          دوستان شما
        </h1>

        <p className="text-blueColor text-xs">
          دوستاتو پیدا کن و باهاشون شرطی بزن !
        </p>
      </div>

      <SearchFriends />

      <div className="w-full flex flex-col gap-3 -mt-1.5">
        <div className="flex items-center gap-6">
          <div className="flex items-center gap-1">
            <h2 className="bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent">
              دوستان شما
            </h2>
            <div className="size-5 text-xs bg-blueColor flex justify-center items-center rounded-lg">
              {toFarsiNumber(data?.friendsList.length)}
            </div>
          </div>

          <FriendRequests data={data?.friendshipRequests} />
        </div>

        {data?.friendsList?.length ? (
          data?.friendsList.map((item) => (
            <FriendItem key={item._id} userInfo={item} />
          ))
        ) : (
          <div className="w-full flex justify-center items-center bg-secondaryDarkTheme h-14 rounded-2xl text-xs">
            <span>دوستی اد نکرده اید !</span>
          </div>
        )}
      </div>
    </div>
  );
};

export default page;
