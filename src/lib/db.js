const mongoose = require("mongoose");

const connectDB = async () => {
  try {
    if (mongoose.connections[0].readyState) {
      return true;
    } else {
      await mongoose.connect(process.env.MONGO_URL);
      console.log("✅ MongoDB Connected");
    }
  } catch (error) {
    console.error("❌ MongoDB Connection Error:", error);
  }
};

module.exports = connectDB;
