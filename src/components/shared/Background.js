import React from "react";

const Background = () => {
  return (
    <>
      <div className="absolute top-[-100px] left-[-100px] w-80 h-80 bg-blueColor rounded-full blur-[10rem] opacity-40"></div>

      <div className="absolute bottom-[-100px] right-[-100px] w-96 h-96 bg-blueColor rounded-full blur-[10rem] opacity-40"></div>
    </>
  );
};

export default Background;
