
export default function Parameter(props) {
    return (
        <li>
            <strong>{props.title}</strong>
            <input
                {...props}
            />
            <span className="units">{props.units}</span>
        </li>
    );
}
