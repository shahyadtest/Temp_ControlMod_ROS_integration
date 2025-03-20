"use client";

import { getData } from "@/services/API";
import { useUser } from "@/store/useUser";
import React, { useEffect } from "react";

const GetUser = () => {
  const { setUser } = useUser();

  useEffect(() => {
    // get user info
    getData("/user/get-info", {})
      .then((res) => {
        setUser(res.data.user);
      })
      .catch((err) => {
        setLoading(false);
      });
  }, []);

  return null;
};

export default GetUser;
