import React, { ReactNode } from "react";

interface RowProps {
    children: ReactNode;
    width?: string;
    height?: string;
}

export default function Row({ children, width = "", height = "" }: RowProps) {
    return (
        <div
            style={{
                display: "flex",
                flexDirection: "row",
                flexGrow: "1 1 auto",
                width: width === "" ? "auto" : width,
                height: height === "" ? "auto" : height,
                justifyContent: "space-between",
            }}
        >
            {children}
        </div>
    );
}