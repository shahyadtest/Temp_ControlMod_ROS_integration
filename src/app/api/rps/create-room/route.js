import { NextResponse } from "next/server";
import connectDB from "@/lib/db";
import RockPaperScissors from "@/models/RockPaperScissors";

export async function POST(req) {
  try {
    await connectDB();
    const { roomId, player1, player2 } = await req.json();

    const newRoom = await RockPaperScissors.create({
      roomId,
      player1,
      player2,
    });

    await newRoom.save();

    return NextResponse.json(
      { message: "اتاق بازی با موفقیت ساخته شد", roomId },
      { status: 200 }
    );
  } catch (error) {
    console.error("خطا هنگام ساخت اتاق:", error);
    return NextResponse.json({ error: "مشکلی رخ داده است." }, { status: 500 });
  }
}
