"use client";

import { getData } from "@/services/API";
import { useUser } from "@/store/useUser";
import { useRouter } from "next/navigation";
import React, { useEffect } from "react";

const GetUserInfo = ({userInfo}) => {
  const { setUser } = useUser();
  const router = useRouter();

  useEffect(() => {
    // get user info
    setUser(userInfo.user);

    if (!userInfo.user) {
      router.push("/auth");
    }
  }, []);

  return null;
};

export default GetUserInfo;
