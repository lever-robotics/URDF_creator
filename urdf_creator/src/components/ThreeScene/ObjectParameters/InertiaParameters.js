import React, { useState, useEffect } from 'react';

function InertiaParameters({ selectedObject, onUpdate }) {
    const [Ixx, setIxx] = useState('');
    const [Ixy, setIxy] = useState('');
    const [Ixz, setIxz] = useState('');
    const [Iyy, setIyy] = useState('');
    const [Iyz, setIyz] = useState('');
    const [Izz, setIzz] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setIxx(selectedObject.userData.Ixx || '');
            setIxy(selectedObject.userData.Ixy || '');
            setIxz(selectedObject.userData.Ixz || '');
            setIyy(selectedObject.userData.Iyy || '');
            setIyz(selectedObject.userData.Iyz || '');
            setIzz(selectedObject.userData.Izz || '');
        }
    }, [JSON.stringify(selectedObject.userData)]);

    const handleInertiaChange = (prop, value) => {
        const updatedObject = { ...selectedObject };
        updatedObject.userData[prop] = parseFloat(value) || 0;
        onUpdate(updatedObject);
    };

    return (
        <div>
            <strong>Moment of Inertia:</strong>
            <ul>
                <li>
                    Ixx:
                    <input
                        type="number"
                        value={Ixx}
                        onChange={(e) => setIxx(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixx', Ixx)}
                    />
                </li>
                <li>
                    Ixy:
                    <input
                        type="number"
                        value={Ixy}
                        onChange={(e) => setIxy(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixy', Ixy)}
                    />
                </li>
                <li>
                    Ixz:
                    <input
                        type="number"
                        value={Ixz}
                        onChange={(e) => setIxz(e.target.value)}
                        onBlur={() => handleInertiaChange('Ixz', Ixz)}
                    />
                </li>
                <li>
                    Iyy:
                    <input
                        type="number"
                        value={Iyy}
                        onChange={(e) => setIyy(e.target.value)}
                        onBlur={() => handleInertiaChange('Iyy', Iyy)}
                    />
                </li>
                <li>
                    Iyz:
                    <input
                        type="number"
                        value={Iyz}
                        onChange={(e) => setIyz(e.target.value)}
                        onBlur={() => handleInertiaChange('Iyz', Iyz)}
                    />
                </li>
                <li>
                    Izz:
                    <input
                        type="number"
                        value={Izz}
                        onChange={(e) => setIzz(e.target.value)}
                        onBlur={() => handleInertiaChange('Izz', Izz)}
                    />
                </li>
            </ul>
        </div>
    );
}

export default InertiaParameters;
