import React, { ReactNode } from "react";
import styles from "./ScreenTools.module.css";

interface RowProps {
    children: ReactNode;
    width?: string;
    height?: string;
}

export default function Row({ children, width = "", height = "" }: RowProps) {
    return (
        <div
            className={styles.row}
            style={{
                width: width === "" ? "auto" : width,
                height: height === "" ? "auto" : height,
            }}>
            {children}
        </div>
    );
}
