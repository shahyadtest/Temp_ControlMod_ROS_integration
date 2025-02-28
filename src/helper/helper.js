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

const getFileSize = (bytes, decimals = 2) => {
  if (bytes === 0) return "0 Bytes";
  const k = 1024;
  const dm = decimals < 0 ? 0 : decimals;
  const sizes = ["بایت", "کیلوبایت", "مگابایت", "گیگابایت"];
  const i = Math.floor(Math.log(bytes) / Math.log(k));
  return parseFloat((bytes / Math.pow(k, i)).toFixed(dm)) + " " + sizes[i];
};

const convertDigits = (num, to = "fa", separate = false) => {
  if (!num) return ""; // جلوگیری از کرش در مقدار null یا undefined
  const enDigits = "0123456789";
  const faDigits = "۰۱۲۳۴۵۶۷۸۹";

  let convertedNum = num
    .split("")
    .map((char) => {
      if (to === "fa") {
        let index = enDigits.indexOf(char);
        return index !== -1 ? faDigits[index] : char;
      } else {
        let index = faDigits.indexOf(char);
        return index !== -1 ? enDigits[index] : char;
      }
    })
    .join("");

  if (separate) {
    convertedNum = convertedNum.replace(/\B(?=(\d{3})+(?!\d))/g, ",");
  }

  return convertedNum;
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

export { convertDigits, getGreeting, getFileSize, toFarsiNumber, idGenerator };
