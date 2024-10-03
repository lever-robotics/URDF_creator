import React from "react";
import styles from "./MenuModal.module.css";

interface MenuItemProps {
    action: () => void;
    label: string;
}

const MenuItem: React.FC<MenuItemProps> = ({ action, label }) => {
    return (
        <li
            className={styles.menuItem}
            onClick={action}>
            {label}
        </li>
    );
};

export default MenuItem;