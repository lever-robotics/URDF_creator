import React, { useState } from 'react';

const ToggleSection = ({ title, children }) => {
    const [isOpen, setIsOpen] = useState(false);

    return (
        <div className="toggle-section">
            <div className={`toggle-header ${isOpen ? 'open' : ''}`} onClick={() => setIsOpen(!isOpen)}>
                <span className="toggle-icon">{isOpen ? '▼' : '▶'}</span>
                <h4>{title}</h4>
            </div>
            {isOpen && <div className="toggle-content">{children}</div>}
        </div>
    );
};

export default ToggleSection;
