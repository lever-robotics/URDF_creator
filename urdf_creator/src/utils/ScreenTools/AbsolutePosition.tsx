import React, { ReactNode } from "react";

type Props = {
    children: ReactNode;
};

export default function AbsolutePosition({ children }: Props) {
    return (
        <div
            style={{
                position: "absolute",
                pointerEvents: "none",
                display: "flex",
                width: "100%",
                height: "100%",
                overflow: "hidden",
            }}
        >
            {children}
        </div>
    );
}
