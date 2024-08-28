import React from "react";
import "./MenuModal.css";

const MenuItem = ({ index, action, label }) => {

    return (
        <li
            key={index}
            className="menu-item"
            onClick={action}>
            {label}
        </li>
    );
};

export default MenuItem;