import React from "react";
import "./MenuModal.css";

const MenuItem = ({ action, label }) => {

    return (
        <li
            className="menu-item"
            onClick={action}>
            {label}
        </li>
    );
};

export default MenuItem;