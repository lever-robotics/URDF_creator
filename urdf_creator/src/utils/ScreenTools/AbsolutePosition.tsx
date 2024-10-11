import type { ReactNode } from "react";
import styles from "./ScreenTools.module.css";

type Props = {
    children: ReactNode;
};

export default function AbsolutePosition({ children }: Props) {
    return <div className={styles.absolutePosition}>{children}</div>;
}
