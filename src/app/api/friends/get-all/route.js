import connectDB from "@/lib/db";
import FriendRequest from "@/models/FriendRequest";
import Friendship from "@/models/Friendship";
import jwt from "jsonwebtoken";
import { NextResponse } from "next/server";

export async function GET(req) {
  try {
    await connectDB();

    // get user token
    const token =
      req.cookies.get("token")?.value ||
      req.headers.get("authorization").split(" ")[1];
    if (!token) {
      return NextResponse.json(
        { error: "توکن یافت نشد.", auth },
        { status: 401 }
      );
    }

    // get user id by token
    let userId;
    try {
      const decoded = jwt.verify(token, process.env.JWT_SECRET);
      userId = decoded.userId;
    } catch (err) {
      return NextResponse.json({ error: "توکن نامعتبر است." }, { status: 401 });
    }

    // get user friends
    const friends = await Friendship.find({
      $or: [{ user1: userId }, { user2: userId }],
    }).populate("user1 user2", "userName nickName");

    const friendList = friends.map((friend) => {
      return friend.user1._id.toString() === userId
        ? friend.user2
        : friend.user1;
    });

    // find users friendship requests
    const friendRequests = await FriendRequest.find({
      receiver: userId,
      status: "pending", // return requests with pending status
    }).populate("sender receiver", "userName nickName");

    return NextResponse.json({
      message: "درخواست با موفقیت انجام شد.",
      friendsList: friendList,
      friendshipRequests: friendRequests,
    });
  } catch (error) {
    console.error("خطا در دریافت اطلاعات:", error);
    return NextResponse.json({ error: "مشکلی رخ داده است." }, { status: 500 });
  }
}
