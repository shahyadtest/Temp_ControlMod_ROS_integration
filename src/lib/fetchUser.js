"use server";
import { baseURL } from "@/services/API";
import { cookies } from "next/headers";
import "server-only";

export async function getUser() {
  const token = (await cookies()).get("token")?.value;

  if (!token) {
    return null;
  }

  try {
    const userRes = await fetch(`${baseURL}/user/get-info`, {
      method: "GET",
      headers: {
        Authorization: `Bearer ${token}`,
      },
    });

    const userData = await userRes.json();

    return userData;
  } catch (error) {
    console.log(error);
  }
}
