import { NextResponse } from "next/server";
import RockPaperScissors from "@/models/RockPaperScissors";
import connectDB from "@/lib/db";

export async function POST(req) {
  await connectDB(); // اضافه کردن await

  try {
    const { roomId, moves, winner } = await req.json();

    if (!roomId || !Array.isArray(moves) || !moves.length) {
      return NextResponse.json(
        { error: "roomId and moves are required" },
        { status: 400 }
      );
    }

    const updatedGame = await RockPaperScissors.findOneAndUpdate(
      { roomId },
      {
        moves,
        winner: winner || null,
      },
      { new: true }
    );

    if (!updatedGame) {
      return NextResponse.json({ error: "Game not found" }, { status: 404 });
    }

    return NextResponse.json(
      {
        success: true,
        game: updatedGame,
        message: "Game result saved successfully",
      },
      { status: 200 }
    );
  } catch (error) {
    console.error("Error saving game result:", error);
    return NextResponse.json({ error: error.message }, { status: 500 });
  }
}
