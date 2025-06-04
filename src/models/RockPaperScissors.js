import mongoose from "mongoose";
const { Schema } = mongoose;

const RockPaperScissorsSchema = new Schema({
  roomId: { type: String, required: true, unique: true },
  player1: { type: String, required: true },
  player2: { type: String, required: true },
  moves: {
    type: [Schema.Types.Mixed], // یا type: [Object]
    default: [],
  },
  winner: { type: String, default: null },
  createdAt: { type: Date, default: () => Date.now() },
});

export default mongoose.models.RockPaperScissors ||
  mongoose.model("RockPaperScissors", RockPaperScissorsSchema);
