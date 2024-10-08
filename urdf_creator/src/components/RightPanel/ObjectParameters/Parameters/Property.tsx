import { ReactNode } from "react";
import styles from "./Parameter.module.css";

export default function Property({
    name,
    children,
}: {
    name?: string;
    children: ReactNode;
}) {
    return (
        <div className={styles.property}>
            {name && <div className={styles.name}>{name}</div>}
            <div className={styles.propertyChildren}>{children}</div>
            {/* <hr className="line"></hr> */}
        </div>
    );
}
