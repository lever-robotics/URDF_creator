import React from "react";
import "./MenuModal.css";

interface MenuItemProps {
    action: () => void;
    label: string;
}

const MenuItem: React.FC<MenuItemProps> = ({ action, label }) => {
    return (
        <li
            className="menu-item"
            onClick={action}>
            {label}
        </li>
    );
};

export default MenuItem;