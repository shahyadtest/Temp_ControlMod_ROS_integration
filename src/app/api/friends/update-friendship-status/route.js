import connectDB from "@/lib/db";
import FriendRequest from "@/models/FriendRequest";
import Friendship from "@/models/Friendship";
import { NextResponse } from "next/server";

export async function POST(req) {
  try {
    await connectDB();

    const { requestId, accept } = await req.json();

    const friendRequest = await FriendRequest.findById(requestId);
    if (!friendRequest) {
      return NextResponse.json(
        { message: "درخواست دوستی یافت نشد." },
        { status: 404 }
      );
    }

    if (accept) {
      // create friendship in Friendship Model
      await Friendship.create({
        user1: friendRequest.sender,
        user2: friendRequest.receiver,
      });

      // update frienship status
      friendRequest.status = "accepted";
    } else {
      friendRequest.status = "rejected";
    }

    await friendRequest.save();

    return NextResponse.json({
      message: accept ? "درخواست دوستی پذیرفته شد!" : "درخواست رد شد.",
    });
  } catch (error) {
    console.error("خطا در دریافت اطلاعات:", error);
    return NextResponse.json({ error: "مشکلی رخ داده است." }, { status: 500 });
  }
}
