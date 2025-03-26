import { NextResponse } from "next/server";
import connectDB from "@/lib/db";
import FriendRequest from "@/models/FriendRequest";

export async function POST(request) {
  try {
    await connectDB();

    const { senderId, receiverId } = await request.json();

    if (!senderId || !receiverId) {
      return NextResponse.json(
        { message: "اطلاعات ناقص است." },
        { status: 400 }
      );
    }

    // check if request exist
    const existingRequest = await FriendRequest.findOne({
      sender: senderId,
      receiver: receiverId,
    });

    if (existingRequest) {
      if (existingRequest.status === "pending") {
        return NextResponse.json(
          { message: "درخواست قبلاً ارسال شده است." },
          { status: 400 }
        );
      } else if (existingRequest.status === "rejected") {
        // if request was rejected send new request
        await FriendRequest.findByIdAndDelete(existingRequest._id);
      }
    }

    // create new friendship request
    await FriendRequest.create({
      sender: senderId,
      receiver: receiverId,
      status: "pending",
    });

    return NextResponse.json(
      { message: "درخواست دوستی ارسال شد!" },
      { status: 200 }
    );
  } catch (error) {
    console.error("خطا در ارسال درخواست دوستی:", error);
    return NextResponse.json({ error: "مشکلی رخ داده است." }, { status: 500 });
  }
}
