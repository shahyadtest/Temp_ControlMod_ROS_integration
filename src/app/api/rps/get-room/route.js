import RockPaperScissors from "@/models/RockPaperScissors";
import { NextResponse } from "next/server";
import connectDB from "@/lib/db";
import User from "@/models/User";

export async function POST(req) {
  connectDB();

  try {
    const { roomId, userId } = await req.json();

    if (!roomId || !userId)
      return NextResponse.json(
        { message: "آیدی اتاق و آیدی کاربر اجباری میباشد" },
        { status: 400 }
      );

    // find room
    const room = await RockPaperScissors.findOne({ roomId });
    if (!room)
      return NextResponse.json(
        { message: "اتاق مورد نظر یافت نشد" },
        { status: 400 }
      );

    // get players info
    const player1 = await User.findOne({ _id: room.player1 }, "-__v -password");
    const player2 = await User.findOne({ _id: room.player2 }, "-__v -password");

    if (!player1 || !player2) {
      return NextResponse.json(
        { message: "بازکن ها یافت نشدند" },
        { status: 400 }
      );
    }

    // recognize user & opponent
    let user, opponent;
    if (room.player1 === userId) {
      user = player1;
      opponent = player2;
    } else if (room.player2 === userId) {
      user = player2;
      opponent = player1;
    } else {
      return NextResponse.json(
        { message: "بازیکن ها در این اتاق نیستند" },
        { status: 400 }
      );
    }

    return NextResponse.json(
      {
        roomId,
        user,
        opponent,
        host: player1,
      },
      { status: 200 }
    );
  } catch (error) {
    return NextResponse.json({ error: error.message }, { status: 500 });
  }
}
