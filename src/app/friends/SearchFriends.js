"use client";

import { getData, postData } from "@/services/API";
import { Input, Spinner } from "@heroui/react";
import Image from "next/image";
import React, { useState } from "react";
import toast, { Toaster } from "react-hot-toast";
import UserItem from "./UserItem";
import { useUser } from "@/store/useUser";

const SearchFriends = () => {
  const { user } = useUser();
  const [search, setSearch] = useState("");
  const [loading, setLoading] = useState(false);
  const [users, setUsers] = useState([]);
  console.log(user);
  const onChangeSearchHandler = (e) => {
    const value = e.target.value;
    setSearch(value);

    if (value.length > 3) {
      setLoading(true);

      postData("/friends/search", { search: value })
        .then((res) => {
          setLoading(false);

          setUsers(res.data.users);
        })
        .catch((err) => {
          toast.error("خطا هنگام جست و جو", {
            style: {
              borderRadius: "10px",
              background: "#040e1c",
              color: "#fff",
              fontSize: "14px",
            },
          });
          setLoading(false);
        });
    }
  };

  return (
    <div className="w-full flex flex-col gap-4">
      <Toaster />

      <Input
        type="text"
        startContent={
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width={16}
            height={16}
            viewBox="0 0 16 16"
          >
            <path
              d="M7.016 0.809376C5.518511999999999 0.914976 4.392736 1.2555999999999998 3.432 1.893808C2.0389120000000003 2.819216 1.184656 4.257328 0.8978400000000001 6.16C0.821728 6.664944 0.808896 6.870496 0.808432 7.5920000000000005C0.808 8.291904 0.815776 8.435072 0.880656 8.92C1.179824 11.156255999999999 2.2786880000000003 12.793408 4.072 13.674719999999999C5.1572320000000005 14.208032 6.4747200000000005 14.452464 7.974624 14.398719999999999C9.203984 14.354688000000001 10.213536 14.124080000000001 11.115343999999999 13.681344C11.489744 13.497520000000002 11.768928 13.327247999999999 12.069711999999999 13.099248L12.195408 13.003968 13.253712 14.058816C14.089360000000001 14.891727999999999 14.328832 15.121728 14.392 15.152032C14.510912000000001 15.209072 14.710432 15.215712 14.84 15.166928C14.972928 15.116896 15.116896 14.972928 15.166928 14.84C15.215712 14.710432 15.209072 14.510912000000001 15.152032 14.392C15.121728 14.328832 14.891727999999999 14.089360000000001 14.058816 13.253712L13.003968 12.195408 13.099248 12.069711999999999C13.562479999999999 11.458608 13.90096 10.751184 14.12016 9.936C14.288448 9.310112 14.371936 8.722288 14.398719999999999 7.974624C14.444656000000002 6.692591999999999 14.27168 5.536784 13.886528 4.5520000000000005C13.344448 3.166016 12.380832 2.121008 11.070448 1.4980799999999999C10.33544 1.1486720000000001 9.570448 0.9464640000000001 8.575520000000001 0.83864C8.358608 0.815136 7.233744 0.7940320000000001 7.016 0.809376M7.176 1.945376C6.126752000000001 2.0077439999999998 5.3565119999999995 2.182864 4.644192 2.52096C4.147008 2.7569600000000003 3.786912 3.0108 3.398864 3.398864C3.0108 3.786912 2.7569600000000003 4.147008 2.52096 4.644192C2.226672 5.264208 2.061296 5.90816 1.9734720000000001 6.776C1.9352639999999999 7.153584 1.9352960000000001 8.060128 1.97352 8.432C2.043216 9.109904 2.163696 9.66296 2.351696 10.168000000000001C2.4468159999999997 10.423536 2.6822880000000002 10.890336 2.827888 11.112C3.5918560000000004 12.275103999999999 4.8019359999999995 12.972848 6.432 13.190144C6.8512 13.246032 7.02456 13.255616 7.6080000000000005 13.2552C8.060416 13.254864 8.264128000000001 13.248048 8.472 13.226304C9.933904 13.073312 10.993216 12.622992 11.808112 11.808112C12.622992 10.993216 13.073312 9.933904 13.226304 8.472C13.261664000000001 8.134144000000001 13.267136 7.129376000000001 13.235327999999999 6.816C13.07968 5.282608000000001 12.638608 4.22608 11.806928000000001 3.394464C10.989904000000001 2.577504 9.902672 2.119408 8.44 1.975824C8.199952 1.9522560000000002 7.388864 1.93272 7.176 1.945376"
              stroke="none"
              fill="currentColor"
              fillRule="evenodd"
              strokeWidth={0.016}
            />
          </svg>
        }
        endContent={loading ? <Spinner size="sm" color="current" /> : ""}
        label={
          <>
            <span>جست و جو</span>
            <span className="text-[11px] text-gray-400 pr-0.5">
              (حداقل سه کاراکتر تایپ کنید)
            </span>
          </>
        }
        value={search}
        onChange={onChangeSearchHandler}
        placeholder="نام نمایشی یا نام کاربری دوست خود را جست و جو کنید..."
        variant="bordered"
        labelPlacement="outside"
        classNames={{
          label: "!text-gray-200 -bottom-0",
          input: "placeholder:text-xs",
          inputWrapper:
            "!bg-secondaryDarkTheme focus-within:!border-borderColor !shadow-none !border-none",
        }}
      />

      <div className="w-full flex flex-col gap-3">
        <h2 className="bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent">
          نتیجه جست و جوی شما
        </h2>

        <div className="w-full flex flex-col gap-2">
          {users.map((item) => (
            <UserItem key={item._id} data={item} />
          ))}
        </div>
      </div>
    </div>
  );
};

export default SearchFriends;
