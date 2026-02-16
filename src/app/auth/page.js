"use client";

import { getData, postData } from "@/services/API";
import { useUser } from "@/store/useUser";
import { Button, Input } from "@heroui/react";
import { Icon } from "@iconify/react";
import Image from "next/image";
import Link from "next/link";
import { useRouter } from "next/navigation";
import React, { useEffect, useState } from "react";
import { useForm } from "react-hook-form";
import toast, { Toaster } from "react-hot-toast";

const page = () => {
  const [showPassword, setShowPassword] = useState(false);
  const [formType, setFormType] = useState("register");
  const [loading, setLoading] = useState(false);
  const router = useRouter();
  const { setUser } = useUser();

  const {
    register,
    handleSubmit,
    setValue,
    control,
    watch,
    reset,
    clearErrors,
    getValues,
    formState: { errors },
  } = useForm({
    mode: "onBlur",
    defaultValues: {
      nickName: "",
      userName: "",
      phoneNumber: "",
      password: "",
    },
  });

  const showPasswordToggle = () => setShowPassword(!showPassword);

  const userRegisterHandler = (data) => {
    setLoading(true);

    postData("/user/register", { ...data })
      .then((res) => {
        toast.success("ثبت نام با موفقیت انجام شد", {
          style: {
            borderRadius: "10px",
            background: "#040e1c",
            color: "#fff",
            fontSize: "14px",
          },
        });

        router.push("/");

        setUser(res.data.user);
      })
      .catch((err) => {
        setLoading(false);
      });
  };

  const userLoginHandler = (data) => {
    setLoading(true);

    postData("/user/login", { ...data })
      .then((res) => {
        toast.success("ورود با موفقیت انجام شد", {
          style: {
            borderRadius: "10px",
            background: "#040e1c",
            color: "#fff",
            fontSize: "14px",
          },
        });

        router.push("/");

        setUser(res.data.user);
      })
      .catch((err) => {
        toast.error(err.response.data.message, {
          style: {
            borderRadius: "10px",
            background: "#040e1c",
            color: "#fff",
            fontSize: "14px",
          },
        });

        setLoading(false);
      });
  };

  return (
    <div className="relative max-w-[450px] flex flex-col items-center justify-center gap-5 w-full min-h-screen bg-primaryDarkTheme overflow-hidden">
      <Toaster />
      <div className="w-full overflow-hidden absolute top-0">
        {/* bottom */}
        <div className="absolute bottom-0 bottom-overly w-full h-full"></div>
        <div className="absolute top-0 bg-gradient-to-b from-primaryDarkTheme w-full h-40"></div>

        <Image
          src={"/auth-bg.webp"}
          className="h-full max-h-[470px] object-cover object-bottom"
          width={1000}
          height={500}
          alt="ورود به حساب کاربری - شطرنج آنلاین"
        />
      </div>

      <div className="relative flex flex-col justify-center items-center z-10 p-5">
        <h1 className="text-4xl font-black leading-[50px] md:leading-[70px] bg-gradient-to-b from-white to-gray-600 bg-clip-text text-transparent">
          گیم هاب
        </h1>

        <p className="text-sm bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent">
          بازی شطرنج آنلاین فرصتی عالی برای رقابت و لذت بردن از یک تجربه شطرنج
          هیجان‌انگیز است. می‌توانید با دوستان خود بازی کنید، رقابت‌های دوستانه
          داشته باشید یا در حالت شرط‌بندی مهارت‌های خود را محک بزنید. این بازی
          با طراحی جذاب و فضای رقابتی، شما را به چالش می‌کشد و لحظات
          هیجان‌انگیزی را برایتان رقم می‌زند.
        </p>

        <p className="text-sm text-blueColor mt-3">
          آماده‌اید که قدرت تفکر خود را در برابر دیگران به چالش بکشید؟
        </p>

        {formType === "register" ? (
          <div className="w-full flex flex-col items-center gap-4 mt-4">
            <h2 className="text-xl font-black bg-gradient-to-b from-white to-gray-600 bg-clip-text text-transparent mt-4">
              ثبت نام و ورود به بازی
            </h2>

            <div className="w-full flex flex-col gap-3">
              <Input
                type="text"
                label="نام نمایشی"
                placeholder="نام نمایشی خود در بازی را وارد کنید"
                variant="bordered"
                labelPlacement="outside"
                classNames={{
                  label: "!text-gray-200 -bottom-0",
                  input: "placeholder:text-xs",
                  inputWrapper:
                    "!bg-secondaryDarkTheme focus-within:!border-borderColor !shadow-none !border-none",
                }}
                isInvalid={errors.nickName ? true : false}
                errorMessage={errors?.nickName?.message}
                {...register("nickName", {
                  validate: {
                    isRequired: (value) =>
                      value.length > 0 || "نام نمایشی اجباری میباشد",
                  },
                })}
              />

              <Input
                type="text"
                label={
                  <>
                    <span>نام کاربری</span>
                    <span className="text-[11px] text-gray-400 pr-1">
                      (نام کاربری بصورت حروف انگلیسی میباشد)
                    </span>
                  </>
                }
                placeholder="نام کاربری خود را وارد کنید"
                variant="bordered"
                labelPlacement="outside"
                classNames={{
                  label: "!text-gray-200 -bottom-0",
                  input: "placeholder:text-xs",
                  inputWrapper:
                    "!bg-secondaryDarkTheme focus-within:!border-borderColor !shadow-none !border-none",
                }}
                isInvalid={errors.userName ? true : false}
                errorMessage={errors?.userName?.message}
                {...register("userName", {
                  validate: {
                    isRequired: (value) =>
                      value.length > 0 || "نام کاربری اجباری میباشد",
                  },
                })}
              />

              <Input
                type="text"
                label={
                  <>
                    <span>شماره تلفن</span>
                    <span className="text-[11px] text-gray-400 pr-1">
                      (اختیاری)
                    </span>
                  </>
                }
                placeholder="شماره تلفن خود را وارد کنید"
                variant="bordered"
                labelPlacement="outside"
                classNames={{
                  label: "!text-gray-200 -bottom-0",
                  input: "placeholder:text-xs",
                  inputWrapper:
                    "!bg-secondaryDarkTheme focus-within:!border-borderColor !shadow-none !border-none",
                }}
                isInvalid={errors.phoneNumber ? true : false}
                errorMessage={errors?.phoneNumber?.message}
                {...register("phoneNumber", {
                  validate: {
                    isRequired: (value) =>
                      value.length > 0 || "شماره تلفن اجباری میباشد",
                    isNumber: (value) =>
                      /^[0-9\b]+$/.test(value) ||
                      "فرمت شماره تلفن صحیح نمیباشد",
                  },
                })}
              />

              <Input
                label="رمز عبور"
                placeholder="رمز عبور خود را وارد کنید"
                variant="bordered"
                labelPlacement="outside"
                classNames={{
                  label: "!text-gray-200 -bottom-0",
                  input: "placeholder:text-xs",
                  inputWrapper:
                    "!bg-secondaryDarkTheme focus-within:!border-borderColor !shadow-none !border-none",
                }}
                endContent={
                  <button
                    className="focus:outline-none"
                    type="button"
                    onClick={showPasswordToggle}
                    aria-label="toggle password visibility"
                  >
                    {showPassword ? (
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        fill="none"
                        viewBox="0 0 24 24"
                        strokeWidth="1.5"
                        stroke="currentColor"
                        className="size-4 text-gray-400"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          d="M3.98 8.223A10.477 10.477 0 0 0 1.934 12C3.226 16.338 7.244 19.5 12 19.5c.993 0 1.953-.138 2.863-.395M6.228 6.228A10.451 10.451 0 0 1 12 4.5c4.756 0 8.773 3.162 10.065 7.498a10.522 10.522 0 0 1-4.293 5.774M6.228 6.228 3 3m3.228 3.228 3.65 3.65m7.894 7.894L21 21m-3.228-3.228-3.65-3.65m0 0a3 3 0 1 0-4.243-4.243m4.242 4.242L9.88 9.88"
                        />
                      </svg>
                    ) : (
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        fill="none"
                        viewBox="0 0 24 24"
                        strokeWidth="1.5"
                        stroke="currentColor"
                        className="size-4 text-gray-400"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          d="M2.036 12.322a1.012 1.012 0 0 1 0-.639C3.423 7.51 7.36 4.5 12 4.5c4.638 0 8.573 3.007 9.963 7.178.07.207.07.431 0 .639C20.577 16.49 16.64 19.5 12 19.5c-4.638 0-8.573-3.007-9.963-7.178Z"
                        />
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          d="M15 12a3 3 0 1 1-6 0 3 3 0 0 1 6 0Z"
                        />
                      </svg>
                    )}
                  </button>
                }
                type={showPassword ? "text" : "password"}
                isInvalid={errors.password ? true : false}
                errorMessage={errors?.password?.message}
                {...register("password", {
                  validate: {
                    isRequired: (value) =>
                      value.length > 0 || "رمز عبور اجباری میباشد",
                    isSixLength: (value) =>
                      value.length > 6 || "رمز عبور باید حداقل شش رقم باشد",
                    isLowercase: (value) =>
                      /[a-z]/g.test(value) ||
                      "رمز عبور شما باید شامل حروف کوچک باشد",
                    isUppercase: (value) =>
                      /[A-Z]/g.test(value) ||
                      "رمز عبور شما باید شامل حروف بزرگ باشد",
                  },
                })}
              />

              <Button
                isLoading={loading}
                onClick={handleSubmit(userRegisterHandler)}
                className="mt-2 !bg-blueColor text-white !shadow-none"
              >
                ثبت نام و ورود
              </Button>

              <button
                onClick={() => setFormType("login")}
                className="text-sm bg-gradient-to-b from-white to-gray-400 bg-clip-text text-transparent"
              >
                از قبل حساب دارید ؟ وارد شوید
              </button>
            </div>
          </div>
        ) : (
          <div className="w-full flex flex-col items-center gap-4 mt-4">
            <h2 className="text-xl font-black bg-gradient-to-b from-white to-gray-600 bg-clip-text text-transparent mt-4">
              ورود به بازی
            </h2>

            <div className="w-full flex flex-col gap-3">
              <Input
                type="text"
                label={
                  <>
                    <span>نام کاربری</span>
                  </>
                }
                placeholder="نام کاربری خود را وارد کنید"
                variant="bordered"
                labelPlacement="outside"
                classNames={{
                  label: "!text-gray-200 -bottom-0",
                  input: "placeholder:text-xs",
                  inputWrapper:
                    "!bg-secondaryDarkTheme focus-within:!border-borderColor !shadow-none !border-none",
                }}
                isInvalid={errors.userName ? true : false}
                errorMessage={errors?.userName?.message}
                {...register("userName", {
                  validate: {
                    isRequired: (value) =>
                      value.length > 0 || "نام کاربری اجباری میباشد",
                  },
                })}
              />

              <Input
                label="رمز عبور"
                placeholder="رمز عبور خود را وارد کنید"
                variant="bordered"
                labelPlacement="outside"
                classNames={{
                  label: "!text-gray-200 -bottom-0",
                  input: "placeholder:text-xs",
                  inputWrapper:
                    "!bg-secondaryDarkTheme focus-within:!border-borderColor !shadow-none !border-none",
                }}
                endContent={
                  <button
                    className="focus:outline-none"
                    type="button"
                    onClick={showPasswordToggle}
                    aria-label="toggle password visibility"
                  >
                    {showPassword ? (
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        fill="none"
                        viewBox="0 0 24 24"
                        strokeWidth="1.5"
                        stroke="currentColor"
                        className="size-4 text-gray-400"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          d="M3.98 8.223A10.477 10.477 0 0 0 1.934 12C3.226 16.338 7.244 19.5 12 19.5c.993 0 1.953-.138 2.863-.395M6.228 6.228A10.451 10.451 0 0 1 12 4.5c4.756 0 8.773 3.162 10.065 7.498a10.522 10.522 0 0 1-4.293 5.774M6.228 6.228 3 3m3.228 3.228 3.65 3.65m7.894 7.894L21 21m-3.228-3.228-3.65-3.65m0 0a3 3 0 1 0-4.243-4.243m4.242 4.242L9.88 9.88"
                        />
                      </svg>
                    ) : (
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        fill="none"
                        viewBox="0 0 24 24"
                        strokeWidth="1.5"
                        stroke="currentColor"
                        className="size-4 text-gray-400"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          d="M2.036 12.322a1.012 1.012 0 0 1 0-.639C3.423 7.51 7.36 4.5 12 4.5c4.638 0 8.573 3.007 9.963 7.178.07.207.07.431 0 .639C20.577 16.49 16.64 19.5 12 19.5c-4.638 0-8.573-3.007-9.963-7.178Z"
                        />
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          d="M15 12a3 3 0 1 1-6 0 3 3 0 0 1 6 0Z"
                        />
                      </svg>
                    )}
                  </button>
                }
                type={showPassword ? "text" : "password"}
                isInvalid={errors.password ? true : false}
                errorMessage={errors?.password?.message}
                {...register("password", {
                  validate: {
                    isRequired: (value) =>
                      value.length > 0 || "رمز عبور اجباری میباشد",
                  },
                })}
              />

              <Button
                isLoading={loading}
                onClick={handleSubmit(userLoginHandler)}
                className="mt-2 !bg-blueColor text-white !shadow-none"
              >
                ورود
              </Button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default page;
