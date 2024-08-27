import React from "react";
import "./MenuModal.css"; // Updated styling file

const MenuModal = ({ isOpen, onClose, menuItems, position }) => {
    if (!isOpen) return null;

    return (
        <div
            className="menu-modal"
            style={{ top: position.top, left: position.left }}>
            <button className="close-button" onClick={onClose}>
                &times;
            </button>
            <ul className="menu-list">
                {menuItems.map((item, index) => (
                    <li
                        key={index}
                        className="menu-item"
                        onClick={() => item.action()}>
                        {item.label}
                    </li>
                ))}
            </ul>
        </div>
    );
};

export default MenuModal;
