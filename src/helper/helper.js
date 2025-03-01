const idGenerator = () => Math.random().toString(36).substring(2, 8);

const toFarsiNumber = (num) => {
  const farsiDigits = ["۰", "۱", "۲", "۳", "۴", "۵", "۶", "۷", "۸", "۹"];

  if (num) {
    return num
      .toLocaleString()
      .toString()
      .replace(/\d/g, (x) => farsiDigits[x]);
  } else {
    return (0)
      .toLocaleString()
      .toString()
      .replace(/\d/g, (x) => farsiDigits[x]);
  }
};

const getGreeting = () => {
  const hour = new Date().getHours(); // گرفتن ساعت فعلی سیستم

  if (hour >= 5 && hour < 12) {
    return "صبح بخیر! ☀️";
  } else if (hour >= 12 && hour < 17) {
    return "ظهر بخیر! 🌤️";
  } else if (hour >= 17 && hour < 21) {
    return "عصر بخیر! 🌇";
  } else {
    return "شب بخیر! 🌙";
  }
};

const getSquareColor = (square) => {

  const file = square.charAt(0);
  const rank = square.charAt(1);


  const fileNumber = file.charCodeAt(0) - "a".charCodeAt(0) + 1;


  const sum = fileNumber + parseInt(rank);


  return sum % 2 === 0 ? "black" : "white";
};

export { getGreeting, toFarsiNumber, getSquareColor,idGenerator };
