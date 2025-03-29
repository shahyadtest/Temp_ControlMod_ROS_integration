"use client";

import { getData } from "@/services/API";
import { useUser } from "@/store/useUser";
import { useRouter } from "next/navigation";
import React, { useEffect } from "react";

const GetUserInfo = () => {
  const { setUser } = useUser();
  const router = useRouter();

  useEffect(() => {
    // get user info
    getData("/user/get-info", {})
      .then((res) => {
        setUser(res.data.user);

        if (!res.data.user) {
          router.push("/auth");
        }
      })
      .catch((err) => {
        router.push("/auth");
      });
  }, []);

  return null;
};

export default GetUserInfo;
